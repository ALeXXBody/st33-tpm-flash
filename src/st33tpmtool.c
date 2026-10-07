/* st33tpmtool.c - native Win32 GUI + CLI flash/repair tool for the ST33HTPH2X32
 * (ST33TPHF2XSPI family) TPM 2.0 chip on a Dell XPS 15 9510 (chip U32, SPI CS#2).
 *
 * Transport:  CH341A USB<->SPI adapter (vendor CH341DLL.dll from WCH).
 * Protocol:   TCG PTP 2.0 FIFO/TIS over SPI - byte format as used by the Linux
 *             kernel tpm_tis_spi driver:
 *               READ  : { 0x80|(n-1), 0xD4, addr16>>8, addr16 & 0xFF }
 *                       wait-state bytes (bit0=0), 'ready' token (bit0=1), data
 *               WRITE : { (n-1),      0xD4, addr16>>8, addr16 & 0xFF } + data
 *               n <= 64 bytes data phase per CS assertion (FIFO auto-advances)
 *
 * Commands used:
 *   TPM2_GetCapability(FIRMWARE_VERSION_1)   CC 0x0000017B
 *   TPM2_FieldUpgradeData(UINT16 len, bytes) CC 0x0000019E
 *
 * A built-in simulator mimics ST33 behaviour, including the exact "FU mode"
 * failure seen on the affected laptop, so you can test without hardware.
 *
 * Usage:
 *   st33tpmtool.exe                           -> GUI
 *   st33tpmtool.exe --cli info                -> console probe (no hardware needed!)
 *   st33tpmtool.exe --cli --sim fu_stuck info -> demo the broken-chip state
 *   st33tpmtool.exe --cli fu --file F.bin     -> console flash
 *   st33tpmtool.exe --cli test                -> self-test against simulator
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* --------------------------------------------------------------- constants */
#define MAX_SPI_FRAMESIZE   64u
#define HDR_SIZE             4u

#define REG_ACCESS      0x0000u
#define REG_INT_VECTOR  0x0004u
#define REG_INT_ENABLE  0x0008u
#define REG_INT_STATUS  0x000Cu
#define REG_INTF_CAP    0x0010u
#define REG_STS         0x0018u
#define REG_DATA_FIFO   0x0024u
#define REG_DID_VID     0x0F00u
#define REG_RID         0x0F04u

#define ACCESS_ACTIVE       0x02u
#define ACCESS_REQ_USE      0x08u
#define ACCESS_RELEASE      0x20u

#define STS_VALID       0x80u
#define STS_DATA_AVAIL  0x10u
#define STS_GO          0x20u
#define STS_CMD_READY   0x40u

#define CC_GET_CAPABILITY        0x0000017Bul
#define CC_FIELD_UPGRADE_DATA    0x0000019Eul
#define CAP_TPM_PROPERTIES       0x00000006ul
#define PT_FIRMWARE_VERSION_1    0x00000106ul
#define ST_NO_SESSIONS           0x8001u
#define RC_SUCCESS               0x00000000ul

#define VID_STMICRO 0x104Au
#define VID_NUVOTON 0x1050u
#define VID_ATMEL   0x15D1u
#define VID_INTEL   0x8086u

static const char *vid_name(unsigned vid)
{
    if (vid == VID_STMICRO) return "STMicroelectronics";
    if (vid == VID_NUVOTON) return "Nuvoton";
    if (vid == VID_ATMEL)   return "Atmel/Microchip";
    if (vid == VID_INTEL)   return "Intel (PTT)";
    return "(unknown)";
}

/* ---------------------------------------------------------------- logging */
static int         g_verbose   = 0;
static int         g_gui       = 0;          /* 1 = emit into GUI log         */
static HWND        g_logbox    = NULL;
static FILE       *g_logfile   = NULL;

static void log_line(const char *s)
{
    if (g_gui && g_logbox) {
        int n = (int)SendMessageA(g_logbox, EM_GETLINECOUNT, 0, 0);
        (void)n;
        SendMessageA(g_logbox, EM_SETSEL, (WPARAM)-1, (LPARAM)-1);
        SendMessageA(g_logbox, EM_REPLACESEL, 0, (LPARAM)s);
        SendMessageA(g_logbox, EM_REPLACESEL, 0, (LPARAM)"\r\n");
        SendMessageA(g_logbox, EM_LINESCROLL, 0, (LPARAM)1000);
    } else {
        fputs(s, stdout);
        fputc('\n', stdout);
        fflush(stdout);
    }
    if (g_logfile) {
        fputs(s, g_logfile);
        fputc('\n', g_logfile);
        fflush(g_logfile);
    }
}

static void emit(const char *fmt, ...)
{
    char buf[512];
    va_list a;
    va_start(a, fmt);
    _vsnprintf(buf, sizeof(buf) - 1, fmt, a);
    buf[sizeof(buf) - 1] = 0;
    va_end(a);
    log_line(buf);
}

static void dump_hex(const char *tag, const unsigned char *b, unsigned n)
{
    if (!g_verbose) return;
    char buf[256];
    int  off = _snprintf(buf, sizeof(buf) - 1, "%s (%u):", tag, n);
    for (unsigned i = 0; i < n && off > 0 && off < (int)sizeof(buf) - 4; ++i)
        off += _snprintf(buf + off, sizeof(buf) - (size_t)off - 1, "%s%02X", (i % 8 == 0) ? " " : "", b[i]);
    _snprintf(buf + off, sizeof(buf) - (size_t)off - 1, " ...");
    log_line(buf);
}

/* ------------------------------------------------------------- transport */
typedef int  (*tpm_stream_fn)(unsigned length, unsigned char *buf, void *ctx);
typedef void (*tpm_close_fn)(void *ctx);

typedef struct {
    tpm_stream_fn stream;
    tpm_close_fn  close;
    void *ctx;
} Transport;

/* ------------------------------------------------- CH341A (real hardware) */
typedef int (*ch3_open_t)(unsigned long index);
typedef void (*ch3_close_t)(unsigned long index);
typedef int (*ch3_set_stream_t)(unsigned long index, unsigned long mode);
typedef int (*ch3_stream_spi4_t)(unsigned long index, unsigned long chip,
                                 unsigned long len, unsigned char *buf);

typedef struct {
    HMODULE dll;
    ch3_open_t        open;
    ch3_close_t       close;
    ch3_set_stream_t  set_stream;
    ch3_stream_spi4_t stream_spi4;
    unsigned      index;
    unsigned      mode;
    unsigned      cs;
    unsigned char scratch[HDR_SIZE + MAX_SPI_FRAMESIZE];
} Ch341;

#define CH341_MODE_DEFAULT 0x81u
#define CH341_CS_MASK      0x80u

static void ch341_close(void *ctx)
{
    Ch341 *c = (Ch341 *)ctx;
    if (c->dll) { if (c->close) c->close(c->index); FreeLibrary(c->dll); c->dll = NULL; }
}

static int ch341_stream(unsigned length, unsigned char *buf, void *ctx)
{
    Ch341 *c = (Ch341 *)ctx;
    if (!c->stream_spi4 || length > sizeof(c->scratch)) return 0;
    memcpy(c->scratch, buf, length);
    if (!c->stream_spi4(c->index, c->cs, (unsigned long)length, c->scratch)) return 0;
    memcpy(buf, c->scratch, length);
    return 1;
}

static int ch341_open(Ch341 *c, unsigned index)
{
    memset(c, 0, sizeof(*c));
    c->dll = LoadLibraryA("CH341DLL.dll");
    if (!c->dll) c->dll = LoadLibraryA("CH341DLL");
    if (!c->dll) { emit("ERROR: CH341DLL.dll not found (WCH CH341SER package)."); return 0; }
    c->open        = (ch3_open_t)        GetProcAddress(c->dll, "CH341OpenDevice");
    c->close       = (ch3_close_t)       GetProcAddress(c->dll, "CH341CloseDevice");
    c->set_stream  = (ch3_set_stream_t)  GetProcAddress(c->dll, "CH341SetStream");
    c->stream_spi4 = (ch3_stream_spi4_t) GetProcAddress(c->dll, "CH341StreamSPI4");
    if (!c->open || !c->close || !c->set_stream || !c->stream_spi4) {
        emit("ERROR: CH341DLL.dll missing expected exports.");
        FreeLibrary(c->dll);
        c->dll = NULL;
        return 0;
    }
    c->index = index;
    c->mode  = CH341_MODE_DEFAULT;
    c->cs    = CH341_CS_MASK;
    c->open(index);
    c->set_stream(index, c->mode);
    emit("CH341A: index=%u mode=0x%02X cs=0x%02X", index, c->mode, c->cs);
    return 1;
}

/* -------------------------------------------- built-in simulator (no hw) */
#define SIM_MODE_HEALTHY   0
#define SIM_MODE_FU_STUCK  1
#define SIM_MODE_BUS_DEAD  2

typedef struct {
    int           mode;
    unsigned      access;
    unsigned      sts;
    unsigned char fifo_in[4096];
    unsigned      fifo_in_len;
    unsigned char fifo_out[1024];
    unsigned      fifo_out_len;
    unsigned      fu_next_handle;
    unsigned long long fu_total;
    unsigned      fw_version;
    unsigned char rid;
    unsigned      did;
} Sim;

static void sim_fill_response(Sim *s, const unsigned char *cmd, unsigned cmdlen)
{
    unsigned char *o = s->fifo_out;
    unsigned long cc;
    if (cmdlen < 10) { s->fifo_out_len = 0; return; }
    cc = ((unsigned long)cmd[6] << 24) | ((unsigned long)cmd[7] << 16)
       | ((unsigned long)cmd[8] << 8) | (unsigned long)cmd[9];

    if (s->mode == SIM_MODE_FU_STUCK) {
        /* "TPM is in FU mode": registers answer fine, but the TPM2 command
         * layer never comes up - every command gets a refusal code.         */
        o[0] = 0x80; o[1] = 0x01;
        o[2] = 0; o[3] = 0; o[4] = 0; o[5] = 10;
        o[6] = 0; o[7] = 0; o[8] = 1; o[9] = 1;      /* RC = 0x00000101 FAILURE */
        s->fifo_out_len = 10;
        emit("  [sim] FU mode: command 0x%08lX refused (rc=0x00000101)", cc);
        return;
    }

    if (cc == CC_GET_CAPABILITY) {
        unsigned long cap  = ((unsigned long)cmd[10] << 24) | ((unsigned long)cmd[11] << 16)
                           | ((unsigned long)cmd[12] << 8) | (unsigned long)cmd[13];
        unsigned long prop = ((unsigned long)cmd[14] << 24) | ((unsigned long)cmd[15] << 16)
                           | ((unsigned long)cmd[16] << 8) | (unsigned long)cmd[17];
        if (cap == CAP_TPM_PROPERTIES && prop == PT_FIRMWARE_VERSION_1) {
            o[0] = 0x80; o[1] = 0x01;
            o[2] = 0; o[3] = 0; o[4] = 0; o[5] = 27;
            o[6] = 0; o[7] = 0; o[8] = 0; o[9] = 0;
            o[10] = 1;
            o[11] = 0; o[12] = 0; o[13] = 0; o[14] = 6;
            o[15] = 0; o[16] = 0; o[17] = 0; o[18] = 1;
            o[19] = 0; o[20] = 0; o[21] = 1; o[22] = 6;
            o[23] = (unsigned char)((s->fw_version >> 24) & 0xFF);
            o[24] = (unsigned char)((s->fw_version >> 16) & 0xFF);
            o[25] = (unsigned char)((s->fw_version >> 8) & 0xFF);
            o[26] = (unsigned char)(s->fw_version & 0xFF);
            s->fifo_out_len = 27;
            return;
        }
        goto refuse;
    }
    if (cc == CC_FIELD_UPGRADE_DATA) {
        unsigned len = ((unsigned)cmd[10] << 8) | cmd[11];
        o[0] = 0x80; o[1] = 0x01;
        o[2] = 0; o[3] = 0; o[4] = 0; o[5] = 14;
        o[6] = 0; o[7] = 0; o[8] = 0; o[9] = 0;
        o[10] = (unsigned char)((s->fu_next_handle >> 24) & 0xFF);
        o[11] = (unsigned char)((s->fu_next_handle >> 16) & 0xFF);
        o[12] = (unsigned char)((s->fu_next_handle >> 8) & 0xFF);
        o[13] = (unsigned char)(s->fu_next_handle & 0xFF);
        s->fu_next_handle++;
        s->fu_total += len;
        s->fifo_out_len = 14;
        return;
    }
refuse:
    emit("  [sim] command 0x%08lX refused (%s)", cc,
         s->mode == SIM_MODE_FU_STUCK ? "FU mode" : "unsupported");
    s->fifo_out_len = 0;
}

static void sim_run(Sim *s, unsigned char *buf, unsigned n)
{
    int    is_read = (buf[0] & 0x80) != 0;
    unsigned addr16 = ((unsigned)buf[2] << 8) | buf[3];
    unsigned flow = 1;

    if (s->mode == SIM_MODE_BUS_DEAD) { memset(buf, 0, n); return; }

    if (is_read) {
        unsigned plen = (buf[0] & 0x3F) + 1;
        unsigned char payload[MAX_SPI_FRAMESIZE];
        unsigned pos, k;
        memset(payload, 0, sizeof(payload));

        if (addr16 == REG_DID_VID) {
            /* bytes: DID hi, DID lo, VID hi, VID lo (BE, 4 bytes) */
            /* DID_VID register (TIS): DID little-endian in bytes 0..1, VID LE in 2..3 */
            payload[0] = (unsigned char)(s->did & 0xFF);
            payload[1] = (unsigned char)((s->did >> 8) & 0xFF);
            payload[2] = (unsigned char)(VID_STMICRO & 0xFF);
            payload[3] = (unsigned char)((VID_STMICRO >> 8) & 0xFF);
            flow = 2;
        } else if (addr16 == REG_RID) {
            payload[0] = s->rid;
            flow = 1;
        } else if (addr16 == REG_INTF_CAP) {
            unsigned v = 0x00000015u;
            payload[0] = (unsigned char)((v >> 24) & 0xFF);
            payload[1] = (unsigned char)((v >> 16) & 0xFF);
            payload[2] = (unsigned char)((v >> 8) & 0xFF);
            payload[3] = (unsigned char)(v & 0xFF);
            flow = 1;
        } else if (addr16 == REG_STS) {
            payload[0] = (unsigned char)(s->sts & 0xFF);
            payload[1] = 0x40; payload[2] = 0x00;   /* burst = 64 */
            payload[3] = (unsigned char)((s->sts & STS_DATA_AVAIL) ? 1 : 0);
            flow = 2;
        } else if (addr16 == REG_ACCESS) {
            payload[0] = (unsigned char)(s->access & 0xFF);
            flow = 1;
        } else if (addr16 == REG_DATA_FIFO) {
            unsigned take = s->fifo_out_len < plen ? s->fifo_out_len : plen;
            memset(payload, 0, plen);
            memcpy(payload, s->fifo_out, take);
            memmove(s->fifo_out, s->fifo_out + take, s->fifo_out_len - take);
            s->fifo_out_len -= take;
            if (!s->fifo_out_len) s->sts &= ~STS_DATA_AVAIL;
            flow = 1;
        }

        memset(buf, 0, n);
        for (k = 0; k < flow; ++k) buf[HDR_SIZE + k] = 0x00;
        buf[HDR_SIZE + flow] = 0x01;                    /* ready token */
        pos = HDR_SIZE + flow + 1;
        for (k = 0; k < plen && pos < n; ++k, ++pos) buf[pos] = payload[k];
        return;
    }

    /* ---- write ---- */
    {
        unsigned dlen = (buf[0] & 0x3F) + 1;
        if (addr16 == REG_ACCESS) {
            unsigned char wr = dlen ? buf[HDR_SIZE] : 0;
            if (wr & ACCESS_REQ_USE) s->access |= ACCESS_ACTIVE;
            else if (wr & ACCESS_RELEASE) s->access &= ~ACCESS_ACTIVE;
        } else if (addr16 == REG_STS) {
            unsigned char wr = dlen ? buf[HDR_SIZE] : 0;
            if (wr & STS_CMD_READY) {
                s->sts = STS_VALID | STS_CMD_READY;
                s->fifo_in_len  = 0;
                s->fifo_out_len = 0;
            } else if (wr & STS_GO) {
                emit("   [sim] processing %u-byte command ...", s->fifo_in_len);
                sim_fill_response(s, s->fifo_in, s->fifo_in_len);
                s->fifo_in_len = 0;
                s->sts = s->fifo_out_len ? (STS_VALID | STS_DATA_AVAIL)
                                         : (STS_VALID | STS_CMD_READY);
            }
        } else if (addr16 == REG_DATA_FIFO) {
            if (dlen && s->fifo_in_len + dlen <= sizeof(s->fifo_in)) {
                memcpy(s->fifo_in + s->fifo_in_len, buf + HDR_SIZE, dlen);
                s->fifo_in_len += dlen;
            }
        }
        memset(buf, 0, n);
    }
}

static int sim_stream(unsigned length, unsigned char *buf, void *ctx)
{ sim_run((Sim *)ctx, buf, length); return 1; }

static void sim_close(void *ctx) { (void)ctx; }

/* --------------------------------------------------------- PTP SPI engine */
static int ptp_read(Transport *t, unsigned addr16, unsigned char *out, unsigned n)
{
    unsigned attempt;
    if (n == 0u || n > MAX_SPI_FRAMESIZE) return 1;
    for (attempt = 0; attempt < 5u; ++attempt) {
        unsigned fillers = (unsigned)(8u << attempt);   /* 8,32,128,512... but cap at 200 */
        if (fillers > 200u) fillers = 200u;
        unsigned txlen = HDR_SIZE + fillers + n + 1u;
        unsigned char *sbuf = (unsigned char *)calloc(txlen, 1);
        if (!sbuf) return 1;
        sbuf[0] = 0x80u | (unsigned char)((n - 1) & 0x3F);
        sbuf[1] = 0xD4u;
        sbuf[2] = (unsigned char)((addr16 >> 8) & 0xFF);
        sbuf[3] = (unsigned char)(addr16 & 0xFF);
        dump_hex("ptp_read tx", sbuf, txlen);
        if (!t->stream(txlen, sbuf, t->ctx)) { free(sbuf); return 1; }
        dump_hex("ptp_read rx", sbuf, txlen);
        {
            unsigned idx = HDR_SIZE, consumed = 0;
            if (sbuf[3] & 0x01u) {
                while (idx < txlen && consumed < n) out[consumed++] = sbuf[idx++];
                free(sbuf);
                if (consumed == n) return 0;
                continue;
            }
            while (idx < txlen && !(sbuf[idx] & 0x01u)) idx++;
            if (idx < txlen) {
                idx++;
                while (idx < txlen && consumed < n) out[consumed++] = sbuf[idx++];
                free(sbuf);
                if (consumed == n) return 0;
                continue;
            }
            free(sbuf);
        }
    }
    return 1;
}

static int ptp_write(Transport *t, unsigned addr16, const unsigned char *data, unsigned n)
{
    unsigned char sbuf[HDR_SIZE + MAX_SPI_FRAMESIZE];
    if (n == 0u || n > MAX_SPI_FRAMESIZE) return 1;
    memset(sbuf, 0, sizeof(sbuf));
    sbuf[0] = (unsigned char)((n - 1) & 0x3F);
    sbuf[1] = 0xD4u;
    sbuf[2] = (unsigned char)((addr16 >> 8) & 0xFF);
    sbuf[3] = (unsigned char)(addr16 & 0xFF);
    if (data) memcpy(sbuf + HDR_SIZE, data, n);
    dump_hex("ptp_write tx", sbuf, HDR_SIZE + n);
    if (!t->stream(HDR_SIZE + n, sbuf, t->ctx)) return 1;
    return 0;
}

/* --------------------------------------------------- FIFO/TIS command engine */
typedef struct { Transport *t; } Engine;

static int eng_acquire_locality(Engine *e, unsigned tries)
{
    for (unsigned i = 0; i < tries; ++i) {
        unsigned char va;
        if (ptp_write(e->t, REG_ACCESS, (const unsigned char *)"\x08", 1)) return 1;
        if (ptp_read(e->t, REG_ACCESS, &va, 1)) return 1;
        if (va & ACCESS_ACTIVE) return 0;
        Sleep(5);
    }
    return 1;
}

static int eng_release_locality(Engine *e)
{
    return ptp_write(e->t, REG_ACCESS, (const unsigned char *)"\x20", 1);
}

static unsigned long eng_read_sts(Engine *e, int *ok)
{
    unsigned char b[8];
    *ok = 0;
    if (ptp_read(e->t, REG_STS, b, 4)) return 0;
    *ok = 1;
    return (unsigned long)b[0] | ((unsigned long)b[1] << 8)
         | ((unsigned long)b[2] << 16) | ((unsigned long)b[3] << 24);
}

static unsigned eng_burst_count(Engine *e)
{
    int ok;
    unsigned long sts = eng_read_sts(e, &ok);
    if (!ok || !(sts & STS_VALID)) return 0;
    {
        unsigned burst = (unsigned)((sts >> 8) & 0xFFFFu);
        if (!burst || burst == 0xFFFFu) burst = 256u;
        return burst;
    }
}

static int eng_send_command(Engine *e, const unsigned char *cmd, unsigned cmdlen,
                            unsigned char *resp, unsigned *resplen, unsigned respmax)
{
    unsigned off = 0, burst, got = 0;
    unsigned long sts;
    int ok;

    if (eng_acquire_locality(e, 50u)) { emit("  cannot acquire locality 0"); return 1; }
    if (ptp_write(e->t, REG_STS, (const unsigned char *)"\x40", 1)) { eng_release_locality(e); return 1; }
    for (unsigned w = 0; w < 300u; ++w) {
        sts = eng_read_sts(e, &ok);
        if (ok && (sts & STS_CMD_READY)) break;
        if (w == 299u) { emit("  commandReady never set"); eng_release_locality(e); return 1; }
        Sleep(5);
    }

    burst = eng_burst_count(e);
    if (!burst) { emit("  burstCount invalid"); eng_release_locality(e); return 1; }

    while (off < cmdlen) {
        unsigned chunk = cmdlen - off;
        if (chunk > burst)           chunk = burst;
        if (chunk > MAX_SPI_FRAMESIZE) chunk = MAX_SPI_FRAMESIZE;
        if (ptp_write(e->t, REG_DATA_FIFO, cmd + off, chunk)) { eng_release_locality(e); return 1; }
        emit("   fifo write %u B @%u/%u (burst=%u)", chunk, off, cmdlen, burst);
        off += chunk;
        burst = eng_burst_count(e);
        if (!burst) { emit("  burstCount invalid (mid-write)"); eng_release_locality(e); return 1; }
    }

    if (ptp_write(e->t, REG_STS, (const unsigned char *)"\x20", 1)) { eng_release_locality(e); return 1; }
    for (unsigned w = 0; w < 1500u; ++w) {
        sts = eng_read_sts(e, &ok);
        if (ok && (sts & STS_DATA_AVAIL)) break;
        if (w == 1499u) {
            emit("  no dataAvail (chip silent) - STS=0x%08lX", sts);
            eng_release_locality(e);
            return 1;
        }
        Sleep(10);
    }

    while (got < respmax) {
        unsigned chunk = eng_burst_count(e);
        if (!chunk) break;
        if (chunk > MAX_SPI_FRAMESIZE)    chunk = MAX_SPI_FRAMESIZE;
        if (chunk > respmax - got)        chunk = respmax - got;
        {
            unsigned char rbuf[MAX_SPI_FRAMESIZE];
            if (ptp_read(e->t, REG_DATA_FIFO, rbuf, chunk)) break;
            memcpy(resp + got, rbuf, chunk);
        }
        got += chunk;
        sts = eng_read_sts(e, &ok);
        if (!ok || !(sts & STS_DATA_AVAIL)) break;   /* response fully drained */
    }
    *resplen = got;
    ptp_write(e->t, REG_STS, (const unsigned char *)"\x40", 1);
    eng_release_locality(e);
    return 0;
}

/* ------------------------------------------------------------- TPM2 tools */
static void put_be16(unsigned char *p, unsigned v)
{ p[0] = (unsigned char)((v >> 8) & 0xFF); p[1] = (unsigned char)(v & 0xFF); }

static void put_be32(unsigned char *p, unsigned long v)
{
    p[0] = (unsigned char)((v >> 24) & 0xFF);
    p[1] = (unsigned char)((v >> 16) & 0xFF);
    p[2] = (unsigned char)((v >> 8) & 0xFF);
    p[3] = (unsigned char)(v & 0xFF);
}

static unsigned build_getcap_fwver(unsigned char *b, unsigned blen)
{
    if (blen < 22u) return 0;
    put_be16(b, ST_NO_SESSIONS);
    put_be32(b + 2, 22u);
    put_be32(b + 6, CC_GET_CAPABILITY);
    put_be32(b + 10, CAP_TPM_PROPERTIES);
    put_be32(b + 14, PT_FIRMWARE_VERSION_1);
    put_be32(b + 18, 1u);
    return 22u;
}

static unsigned long rsp_rc(const unsigned char *rsp)
{
    return ((unsigned long)rsp[6] << 24) | ((unsigned long)rsp[7] << 16)
         | ((unsigned long)rsp[8] << 8) | (unsigned long)rsp[9];
}

/* Corroboration check: a real chip answers DID_VID with the same bytes every
 * time; a floating/dead bus gives different noise on each read.
 * Returns 0 and fills vid/did/rid when a consistent chip is present; 1 if not. */
static int probe_chip(Transport *t, unsigned *vid_out, unsigned *did_out, unsigned *rid_out)
{
    unsigned char a[4], b[4];
    unsigned char r;
    int all0 = 1, all_FF = 1;
    memset(a, 0, sizeof(a));
    memset(b, 0, sizeof(b));
    if (ptp_read(t, REG_DID_VID, a, 4)) return 1;
    if (ptp_read(t, REG_DID_VID, b, 4)) return 1;
    if (memcmp(a, b, 4) != 0) return 1;          /* bus noise, not a chip    */
    for (unsigned i = 0; i < 4; ++i) {           /* reject degenerate values  */
        if (a[i] != 0x00) all0 = 0;
        if (a[i] != 0xFF) all_FF = 0;
    }
    if (all0 || all_FF) return 1;
    r = 0;
    (void)ptp_read(t, REG_RID, &r, 1);
    *did_out = (unsigned)a[0] | ((unsigned)a[1] << 8);
    *vid_out = (unsigned)a[2] | ((unsigned)a[3] << 8);
    *rid_out = r;
    return 0;
}

/* ------------------------------------------------------------- CLI actions */
static void cli_info(Transport *t)
{
    unsigned vid = 0, did = 0, rid = 0;
    if (probe_chip(t, &vid, &did, &rid)) {
        emit("no chip response - bus floating / chip not addressed");
        return;
    }
    emit("VID=0x%04X  DID=0x%04X   (%s)", vid, did, vid_name(vid));
    emit("RID=0x%02X", rid & 0xFF);
}

static void cli_status(Transport *t)
{
    unsigned char b[8];
    unsigned long sts;
    if (ptp_read(t, REG_ACCESS, b, 1) == 0)
        emit("TPM_ACCESS = 0x%02X", b[0]);
    else emit("TPM_ACCESS = (failed)");
    if (ptp_read(t, REG_STS, b, 4) == 0) {
        sts = (unsigned long)b[0] | ((unsigned long)b[1] << 8) | ((unsigned long)b[2] << 16)
            | ((unsigned long)b[3] << 24);
        emit("TPM_STS = 0x%08X", (unsigned)sts);
        emit("   valid=%u burst=%u dataAvail=%u commandReady=%u go=%u",
             (unsigned)((b[0] & STS_VALID) ? 1u : 0u),
             (unsigned)((sts >> 8) & 0xFFFFu),
             (unsigned)((sts & STS_DATA_AVAIL) ? 1u : 0u),
             (unsigned)((sts & STS_CMD_READY) ? 1u : 0u),
             (unsigned)((sts & STS_GO) ? 1u : 0u));
    } else emit("TPM_STS = (failed)");
    if (ptp_read(t, REG_INTF_CAP, b, 4) == 0) {
        unsigned long icap = (unsigned long)b[0] << 24 | (unsigned long)b[1] << 16
                           | (unsigned long)b[2] << 8 | (unsigned long)b[3];
        emit("TPM_INTF_CAP = 0x%08X   fifo=%u crb=%u",
             (unsigned)icap, (unsigned)(icap & 1u), (unsigned)((icap >> 1) & 1u));
    } else emit("TPM_INTF_CAP = (failed)");
}

static int cli_caps(Transport *t, Engine *e)
{
    unsigned char cmd[32], rsp[512];
    unsigned n, rspl = sizeof(rsp);
    unsigned long rc;
    if (!(n = build_getcap_fwver(cmd, sizeof(cmd)))) { emit("build failed"); return 1; }
    rc = 0;
    if (eng_send_command(e, cmd, n, rsp, &rspl, sizeof(rsp))) return 1;
    if (rspl < 10u) { emit("short response (%u)", rspl); return 1; }
    rc = rsp_rc(rsp);
    emit("TPM_RC = 0x%08lX", rc);
    if (rc == RC_SUCCESS && rspl >= 27u) {
        unsigned long val = ((unsigned long)rsp[23] << 24) | ((unsigned long)rsp[24] << 16)
                          | ((unsigned long)rsp[25] << 8) | (unsigned long)rsp[26];
        emit("TPM_PT_FIRMWARE_VERSION_1 = 0x%08lX", val);
    }
    return rc == RC_SUCCESS ? 0 : 1;
}

static int cli_fu(Transport *t, Engine *e, const char *file, int dry)
{
    FILE *fp;
    long total;
    unsigned char *fw, cmd[1100], rsp[512];
    unsigned off = 0, rcc = 0, n, rspl;
    size_t chunk;
    (void)t;
    fp = fopen(file, "rb");
    if (!fp) { emit("cannot open %s", file); return 1; }
    fseek(fp, 0, SEEK_END); total = ftell(fp); fseek(fp, 0, SEEK_SET);
    if (total <= 0) { emit("empty file"); fclose(fp); return 1; }
    fw = (unsigned char *)malloc((size_t)total);
    if (!fw) { fclose(fp); return 1; }
    if (fread(fw, 1, (size_t)total, fp) != (size_t)total) { emit("read failed"); fclose(fp); free(fw); return 1; }
    fclose(fp);
    emit("firmware: %s (%ld bytes)", file, total);
    if (dry) { emit("--dry: not transmitting."); free(fw); return 0; }

    while (off < (unsigned)total) {
        chunk = 960u;
        if (chunk > (unsigned)total - off) chunk = (unsigned)total - off;
        n = 12u + (unsigned)chunk;
        cmd[0] = 0x80u; cmd[1] = 0x01u;
        cmd[6] = 0x9Eu; cmd[7] = 0x01u; cmd[8] = 0x00u; cmd[9] = 0x00u;
        cmd[10] = (unsigned char)((chunk >> 8) & 0xFF);
        cmd[11] = (unsigned char)(chunk & 0xFF);
        memcpy(cmd + 12, fw + off, chunk);
        put_be32(cmd + 2, n);
        emit("FU chunk @%u/%u (%u B)", off, (unsigned)total, (unsigned)chunk);
        rspl = sizeof(rsp);
        if (eng_send_command(e, cmd, n, rsp, &rspl, sizeof(rsp))) { free(fw); return 1; }
        if (rspl < 10u) { emit("short reply"); free(fw); return 1; }
        {
            unsigned long rc = rsp_rc(rsp);
            if (rc != RC_SUCCESS) { emit("TPM refused: 0x%08lX", rc); free(fw); return 1; }
        }
        rcc++;
        off += (unsigned)chunk;
    }
    emit("stream complete: %u chunks, %u bytes", rcc, off);
    free(fw);
    return 0;
}

static int cli_test(void)
{
    int pass = 1;
    const char *names[] = { "healthy", "fu_stuck", "bus_dead" };
    int sims[] = { SIM_MODE_HEALTHY, SIM_MODE_FU_STUCK, SIM_MODE_BUS_DEAD };
    for (unsigned i = 0; i < 3u; ++i) {
        Sim        sim;
        Transport  t;
        Engine     e;
        unsigned char b[8];
        unsigned vid, did;
        memset(&sim, 0, sizeof(sim));
        sim.mode = sims[i];
        sim.fw_version = 0x01077100u;
        sim.did = 0x0001u;
        sim.rid = 0xA3u;
        sim.sts = STS_VALID;
        t.stream = sim_stream;
        t.close  = sim_close;
        t.ctx    = &sim;
        e.t = &t;
        emit("=== %s ===", names[i]);
        {
            unsigned rid_v;
            memset(b, 0, sizeof(b));
            vid = did = rid_v = 0;
            if (probe_chip(&t, &vid, &did, &rid_v) == 0)
                emit("  info: VID=0x%04X DID=0x%04X", vid, did);
            else
                emit("  info: (no chip response)");
        }
        if (sims[i] == SIM_MODE_HEALTHY) {
            if (vid != VID_STMICRO) { emit("     FAIL: expected VID 0x104A"); pass = 0; }
            else emit("     ok: DID_VID responds with ST VID");
            if (cli_caps(&t, &e)) { emit("     FAIL: caps"); pass = 0; }
        } else if (sims[i] == SIM_MODE_FU_STUCK) {
            if (vid != VID_STMICRO) { emit("     FAIL: DID_VID should respond"); pass = 0; }
            else emit("     ok: DID_VID responds even frozen in FU mode");
            if (!cli_caps(&t, &e)) { emit("     FAIL: caps should be refused"); pass = 0; }
            else emit("     ok: chip refuses TPM2 commands (FU mode)");
        } else {
            if (vid != 0u) { emit("     FAIL: bus should be dead"); pass = 0; }
            else emit("     ok: chip absent (reads float)");
        }
    }
    emit("");
    emit("%s", pass ? "SELFTEST PASS" : "SELFTEST FAIL");
    return pass ? 0 : 1;
}

/* --------------------------------------------------------------- GUI code */
#define IDC_OPEN        1001L
#define IDC_CLOSE       1002L
#define IDC_PROVIDER    1003L
#define IDC_DEVICEIDX   1004L
#define IDC_PROBE       1005L
#define IDC_STATUS      1006L
#define IDC_CAPS        1007L
#define IDC_FWFILE      1008L
#define IDC_BROWSE      1009L
#define IDC_DRYRUN      1010L
#define IDC_FLASH       1011L
#define IDC_SIMMODE     1012L
#define IDC_CHIPINFO    1013L
#define IDC_LOGB        1014L

typedef struct {
    Sim     sim;
    Ch341   ch341;
    Transport tport;
    Engine  eng;
    int     backend;         /* 0 = hardware, 1 = sim */
    int     open_ok;
} App;

static App g_app;
static unsigned g_sim_mode = SIM_MODE_HEALTHY;
static int  g_gui_sim = 0;
static HWND g_hwnd = NULL;

static void app_apply_backend(void)
{
    if (g_app.backend) {
        memset(&g_app.sim, 0, sizeof(g_app.sim));
        g_app.sim.mode = (int)g_sim_mode;
        g_app.sim.fw_version = 0x01077100u;
        g_app.sim.did = 0x0001u;
        g_app.sim.rid = 0xA3u;
        g_app.sim.sts = STS_VALID;
        g_app.tport.stream = sim_stream;
        g_app.tport.close  = sim_close;
        g_app.tport.ctx    = &g_app.sim;
    } else {
        g_app.tport.stream = ch341_stream;
        g_app.tport.close  = ch341_close;
        g_app.tport.ctx    = &g_app.ch341;
    }
    g_app.eng.t = &g_app.tport;
}

static HFONT   g_fnt;
static BOOL CALLBACK setfont_proc(HWND c, LPARAM lp)
{ (void)lp; SendMessageA(c, WM_SETFONT, (WPARAM)g_fnt, TRUE); return TRUE; }

static LRESULT CALLBACK main_wndproc(HWND h, UINT m, WPARAM wp, LPARAM lp)
{
    switch (m) {
    case WM_CREATE: {
        RECT  r;
        int   W = 760, H = 560;
        int   x1 = 20, y = 20;
        LOGFONTA lf;

        memset(&lf, 0, sizeof(lf));
        lf.lfHeight = -14;
        strcpy_s(lf.lfFaceName, sizeof(lf.lfFaceName), "Segoe UI");
        g_fnt = CreateFontIndirectA(&lf);
        g_logbox = GetDlgItem(h, IDC_LOGB);      /* route emit() output into the UI log */
        if (g_gui_sim)
            SendMessageA(GetDlgItem(h, IDC_SIMMODE), BM_SETCHECK, (WPARAM)BST_CHECKED, 0);

        GetClientRect(h, &r);
        (void)r;

        /* --- column 1: adapter block ----------------------------------------- */
        CreateWindowExA(0, "BUTTON", "Adapter (CH341A)",
                        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                        16, 16, 340, 132, h, (HMENU)0, NULL, NULL);
        CreateWindowExA(0, "STATIC", "Device index:",
                        WS_CHILD | WS_VISIBLE,
                        32, 44, 90, 20, h, NULL, NULL, NULL);
        CreateWindowExA(0, "EDIT", "0",
                        WS_CHILD | WS_VISIBLE | WS_BORDER | ES_NUMBER,
                        128, 42, 60, 22, h, (HMENU)(UINT_PTR)IDC_DEVICEIDX, NULL, NULL);
        CreateWindowExA(0, "BUTTON", "Open",
                        WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                        32, 72, 80, 26, h, (HMENU)(UINT_PTR)IDC_OPEN, NULL, NULL);
        CreateWindowExA(0, "BUTTON", "Close",
                        WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                        120, 72, 80, 26, h, (HMENU)(UINT_PTR)IDC_CLOSE, NULL, NULL);
        CreateWindowExA(0, "BUTTON", "Demo: run simulator instead",
                        WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
                        32, 108, 280, 22, h, (HMENU)(UINT_PTR)IDC_SIMMODE, NULL, NULL);

        /* --- column 2: chip actions ------------------------------------------ */
        CreateWindowExA(0, "BUTTON", "Chip (direct SPI access)",
                        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                        376, 16, 356, 132, h, (HMENU)0, NULL, NULL);
        CreateWindowExA(0, "BUTTON", "Probe",
                        WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                        392, 48, 100, 26, h, (HMENU)(UINT_PTR)IDC_PROBE, NULL, NULL);
        CreateWindowExA(0, "BUTTON", "Status",
                        WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                        500, 48, 100, 26, h, (HMENU)(UINT_PTR)IDC_STATUS, NULL, NULL);
        CreateWindowExA(0, "BUTTON", "Caps",
                        WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                        392, 82, 100, 26, h, (HMENU)(UINT_PTR)IDC_CAPS, NULL, NULL);
        CreateWindowExA(0, "STATIC", "(no chip probed yet)",
                        WS_CHILD | WS_VISIBLE,
                        500, 108, 224, 20, h, (HMENU)(UINT_PTR)IDC_CHIPINFO, NULL, NULL);

        /* --- firmware block --------------------------------------------------- */
        CreateWindowExA(0, "BUTTON", "Firmware payload",
                        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                        16, 156, 716, 92, h, (HMENU)0, NULL, NULL);
        CreateWindowExA(0, "EDIT", "",
                        WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
                        32, 180, 570, 24, h, (HMENU)(UINT_PTR)IDC_FWFILE, NULL, NULL);
        CreateWindowExA(0, "BUTTON", "Browse...",
                        WS_CHILD | WS_VISIBLE | WS_TABSTOP,
                        616, 180, 100, 24, h, (HMENU)(UINT_PTR)IDC_BROWSE, NULL, NULL);
        CreateWindowExA(0, "BUTTON", "Dry run (read only, do not flash)",
                        WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
                        32, 214, 300, 22, h, (HMENU)(UINT_PTR)IDC_DRYRUN, NULL, NULL);

        /* --- flash button ----------------------------------------------------- */
        CreateWindowExA(0, "BUTTON", "FLASH FIRMWARE (one-way, be sure!)",
                        WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
                        16, 258, 716, 34, h, (HMENU)(UINT_PTR)IDC_FLASH, NULL, NULL);

        /* --- log -------------------------------------------------------------- */
        CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", "",
                        WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_MULTILINE |
                        ES_READONLY | ES_AUTOVSCROLL,
                        16, 306, 716, 230, h, (HMENU)(UINT_PTR)IDC_LOGB, NULL, NULL);

        /* apply fonts */
        EnumChildWindows(h, setfont_proc, 0);
        return 0;
    }

    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case IDC_SIMMODE:
            g_app.backend = (SendDlgItemMessageA(h, IDC_SIMMODE, BM_GETCHECK, 0, 0) == BST_CHECKED);
            app_apply_backend();
            emit("backend switched to %s", g_app.backend ? "built-in simulator" : "CH341A hardware");
            return 0;

        case IDC_OPEN:
            if (g_app.backend) {
                emit("simulator backend already active");
            } else {
                char idx[12]; GetDlgItemTextA(h, IDC_DEVICEIDX, idx, sizeof(idx));
                ch341_open(&g_app.ch341, (unsigned)atoi(idx));
                g_app.open_ok = 1;
            }
            return 0;

        case IDC_CLOSE:
            if (g_app.tport.close) g_app.tport.close(g_app.tport.ctx);
            g_app.open_ok = 0;
            emit("adapter closed");
            return 0;

        case IDC_PROBE: {
            unsigned char b[8];
            unsigned vid, did;
            ptp_read(&g_app.tport, REG_DID_VID, b, 4);
            vid = ((unsigned)b[0] << 8) | b[1];
            did = ((unsigned)b[2] << 8) | b[3];
            SetDlgItemTextA(h, IDC_CHIPINFO, vid_name(vid));
            emit("probe: VID=0x%04X DID=0x%04X (%s)", vid, did, vid_name(vid));
            return 0;
        }

        case IDC_STATUS:
            cli_status(&g_app.tport);
            return 0;

        case IDC_CAPS:
            emit("caps: TPM2_GetCapability(FIRMWARE_VERSION_1)...");
            cli_caps(&g_app.tport, &g_app.eng);
            return 0;

        case IDC_BROWSE: {
            char fname[MAX_PATH]; fname[0] = 0;
            OPENFILENAMEA ofn;
            memset(&ofn, 0, sizeof(ofn));
            ofn.lStructSize = sizeof(ofn);
            ofn.hwndOwner = h;
            ofn.lpstrFilter = "TPM Firmware (*.bin)\0*.bin\0All files\0*.*\0";
            ofn.lpstrFile = fname;
            ofn.nMaxFile = sizeof(fname);
            ofn.Flags = OFN_FILEMUSTEXIST;
            if (GetOpenFileNameA(&ofn)) {
                SetDlgItemTextA(h, IDC_FWFILE, fname);
                {
                    /* show file hash on select */
                    FILE *fp = fopen(fname, "rb");
                    if (fp) {
                        fseek(fp, 0, SEEK_END);
                        long sz = ftell(fp);
                        fclose(fp);
                        emit("selected: %s (%ld bytes)", fname, sz);
                    }
                }
            }
            return 0;
        }

        case IDC_FLASH: {
            char fname[MAX_PATH];
            int  dry;
            GetDlgItemTextA(h, IDC_FWFILE, fname, sizeof(fname));
            dry = (SendDlgItemMessageA(h, IDC_DRYRUN, BM_GETCHECK, 0, 0) == BST_CHECKED);
            if (!fname[0]) { emit("flash: pick a firmware file first."); return 0; }
            if (dry) {
                emit("flash (dry run): simulating...");
                cli_fu(&g_app.tport, &g_app.eng, fname, 1);
                return 0;
            }
            if (MessageBoxA(h,
                            "About to write firmware onto the TPM.\n\n"
                            "This action is one-way - the chip's update count is limited,\n"
                            "so use it only once, and only if everything prior looks good.\n\n"
                            "Proceed?",
                            "Confirm", MB_YESNO | MB_ICONWARNING) == IDYES)
            {
                cli_fu(&g_app.tport, &g_app.eng, fname, 0);
            }
            return 0;
        }
        }
        return 0;

    case WM_CTLCOLORBTN:
    case WM_CTLCOLORSTATIC:
        return (LRESULT)GetSysColorBrush(COLOR_WINDOW);

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcA(h, m, wp, lp);
}

/* --------------------------------------------------------------- entrypoint */
static void setup_logging(void)
{
    char path[MAX_PATH];
    char *slash;
    GetModuleFileNameA(NULL, path, sizeof(path));
    slash = strrchr(path, '\\');
    if (slash) *(slash + 1) = 0;
    strcat(path, "st33tpmtool.log");
    g_logfile = fopen(path, "a");
    if (g_logfile) fprintf(g_logfile, "\n===== st33tpmtool on %s ====\n", __DATE__ " " __TIME__);
}

int winmain_gui(HINSTANCE hI, int nShow);

int WINAPI WinMain(HINSTANCE hI, HINSTANCE hP, LPSTR cl, int nShow)
{
    WNDCLASSA wc;
    HWND h;
    MSG  msg;
    int  use_cli = 0, use_sim = 0;
    const char *argv[16];
    int nargc = 0;

    setup_logging();

    /* crude argv from the raw command line */
    {
        char *s;
        (void)cl;
        s = GetCommandLineA();
        /* very simple split */
        while (*s && nargc < 15) {
            while (*s == ' ' || *s == '\t') ++s;
            if (!*s) break;
            argv[nargc++] = s;
            while (*s && *s != ' ' && *s != '\t') ++s;
            if (*s) *s++ = 0;
        }
    }

    for (int i = 1; i < nargc; ++i) {
        if (!strcmp(argv[i], "--cli")) use_cli = 1;
        else if (!strcmp(argv[i], "--sim")) {
            use_sim = 1;
            g_gui_sim = 1;
            if (i + 1 < nargc) {
                if (!strcmp(argv[i + 1], "fu_stuck")) { g_sim_mode = SIM_MODE_FU_STUCK; ++i; }
                else if (!strcmp(argv[i + 1], "bus_dead")) { g_sim_mode = SIM_MODE_BUS_DEAD; ++i; }
                else if (!strcmp(argv[i + 1], "healthy")) { g_sim_mode = SIM_MODE_HEALTHY; ++i; }
            }
        }
        else if (!strcmp(argv[i], "--verbose")) g_verbose = 1;
    }

    /* GUI subsystem has no console of its own. When running in CLI mode from a
     * real terminal, attach to it so logs appear there; when stdout was piped
     * (tests/automation), keep the existing handle instead.                  */
    if (use_cli) {
        HANDLE con = GetStdHandle(STD_OUTPUT_HANDLE);
        if (!con || con == INVALID_HANDLE_VALUE) {
            if (AttachConsole(ATTACH_PARENT_PROCESS)) {
                freopen("CONOUT$", "w", stdout);
                freopen("CONOUT$", "w", stderr);
            }
        }
    }

    if (use_cli) {
        const char *cmd = "?";
        const char *file_arg = NULL;
        int  dry_arg = 0;
        int  simmode = SIM_MODE_HEALTHY;
        int  verb_index = -1;

        for (int i = 1; i < nargc; ++i) {
            if (!strcmp(argv[i], "info") || !strcmp(argv[i], "status") ||
                !strcmp(argv[i], "caps")  || !strcmp(argv[i], "fu")    ||
                !strcmp(argv[i], "test")) { verb_index = i; cmd = argv[i]; break; }
        }
        for (int i = verb_index + 1; i < nargc; ++i) {
            if (!strcmp(argv[i], "--file") && i + 1 < nargc) file_arg = argv[++i];
            else if (!strcmp(argv[i], "--dry")) dry_arg = 1;
        }
        (void)simmode;

        if (use_sim) {
            g_app.backend = 1;
            app_apply_backend();
        } else {
            g_app.backend = 0;
            if (ch341_open(&g_app.ch341, 0)) {
                app_apply_backend();
                g_app.open_ok = 1;
            } else {
                g_gui = 0;
                emit("ERROR: no CH341A adapter available. Retry with '--sim'.");
                return 1;
            }
        }

        if (!strcmp(cmd, "info"))          { cli_info(&g_app.tport);    return 0; }
        else if (!strcmp(cmd, "status"))   { cli_status(&g_app.tport);  return 0; }
        else if (!strcmp(cmd, "caps"))     return cli_caps(&g_app.tport, &g_app.eng);
        else if (!strcmp(cmd, "fu")) {
            if (!file_arg) { emit("fu requires --file <firmware.bin>"); return 1; }
            return cli_fu(&g_app.tport, &g_app.eng, file_arg, dry_arg);
        }
        else if (!strcmp(cmd, "test"))     return cli_test();
        else {
            emit("unknown command (supported: info, status, caps, fu, test)");
            return 1;
        }
    }

    return winmain_gui(hI, nShow);
}

int winmain_gui(HINSTANCE hI, int nShow)
{
    WNDCLASSA wc;
    HWND h;
    MSG  msg;

    g_gui = 1;
    memset(&wc, 0, sizeof(wc));
    wc.lpfnWndProc = main_wndproc;
    wc.hInstance = hI;
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.lpszClassName = "ST33TPMTOOL";
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    RegisterClassA(&wc);

    g_app.backend = g_gui_sim;   /* hardware by default; --sim pre-selects the demo */
    app_apply_backend();

    h = CreateWindowExA(0, "ST33TPMTOOL", "ST33 TPM - field-update tool",
                        WS_OVERLAPPEDWINDOW,
                        CW_USEDEFAULT, CW_USEDEFAULT, 760, 560,
                        NULL, NULL, hI, NULL);
    g_hwnd = h;
    ShowWindow(h, nShow);
    UpdateWindow(h);

    while (GetMessageA(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
    return (int)msg.wParam;
}
