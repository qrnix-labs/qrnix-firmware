// Wire contract v1 emission (ADR-0003) — see serial_contract.h.
//
// Pure C (snprintf only): no Arduino, no malloc. Truncation is clamped:
// the builder never writes past `cap - 1` and always NUL-terminates.

#include "serial_contract.h"

#include <stdarg.h>
#include <stdio.h>

// ── Builder ───────────────────────────────────────────────────────────────────

typedef struct {
    char *p;
    size_t cap;
    size_t n;
} Builder;

static void b_init(Builder *b, char *buf, size_t cap) {
    b->p = buf;
    b->cap = cap;
    b->n = 0;
    if (cap > 0) {
        buf[0] = '\0';
    }
}

static void b_raw(Builder *b, const char *s) {
    while (*s != '\0' && b->n + 1 < b->cap) {
        b->p[b->n++] = *s++;
    }
    if (b->n < b->cap) {
        b->p[b->n] = '\0';
    }
}

static void b_fmt(Builder *b, const char *fmt, ...) {
    if (b->n + 1 >= b->cap) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(b->p + b->n, b->cap - b->n, fmt, ap);
    va_end(ap);
    if (w < 0) {
        return;
    }
    if ((size_t)w >= b->cap - b->n) {
        b->n = b->cap - 1;  // truncated; buffer stays NUL-terminated
    } else {
        b->n += (size_t)w;
    }
}

// Emit `"<escaped>"`. Escapes `"` `\` and control bytes; printable ASCII
// and UTF-8 continuation bytes pass through.
static void b_str(Builder *b, const char *s) {
    b_raw(b, "\"");
    for (const char *c = s; *c != '\0'; c++) {
        const unsigned char ch = (unsigned char)*c;
        char esc = 0;
        switch (ch) {
            case '"':  esc = '"';  break;
            case '\\': esc = '\\'; break;
            case '\n': esc = 'n';  break;
            case '\r': esc = 'r';  break;
            case '\t': esc = 't';  break;
            default:   break;
        }
        if (esc != 0) {
            char pair[3] = {'\\', esc, '\0'};
            b_raw(b, pair);
        } else if (ch < 0x20) {
            char hex[7];
            snprintf(hex, sizeof(hex), "\\u%04x", ch);
            b_raw(b, hex);
        } else {
            char one[2] = {(char)ch, '\0'};
            b_raw(b, one);
        }
    }
    b_raw(b, "\"");
}

// ── Envelope builders ─────────────────────────────────────────────────────────

size_t contract_status_line(char *buf, size_t cap, const ContractStatus *s) {
    Builder b;
    b_init(&b, buf, cap);
    b_raw(&b, "{\"t\":\"status\",\"cv\":");
    b_fmt(&b, "%d", WIRE_CONTRACT_VERSION);
    b_raw(&b, ",\"m\":");
    b_fmt(&b, "%d", s->mode);
    b_raw(&b, ",\"src\":");
    {
        char src[2] = {s->src, '\0'};
        b_str(&b, src);
    }
    b_raw(&b, ",\"red\":");
    b_fmt(&b, "%d", s->red);
    b_raw(&b, ",\"sm\":");
    b_fmt(&b, "%d", s->sm);
    b_raw(&b, ",\"wh\":");
    b_fmt(&b, "%d", s->wh);
    b_raw(&b, ",\"ag\":");
    b_fmt(&b, "%d", s->ag);
    b_raw(&b, ",\"lk\":");
    b_fmt(&b, "%d", s->lk);
    b_raw(&b, ",\"tk\":");
    b_fmt(&b, "%d", s->tk);
    b_raw(&b, ",\"pp\":");
    b_fmt(&b, "%d", s->pp);
    b_raw(&b, ",\"clip\":");
    b_fmt(&b, "%d", s->clip);
    b_raw(&b, ",\"blk_l\":");
    b_fmt(&b, "%lu", (unsigned long)s->blk_l);
    b_raw(&b, ",\"blk_r\":");
    b_fmt(&b, "%lu", (unsigned long)s->blk_r);
    b_raw(&b, ",\"in_l\":");
    b_fmt(&b, "%lu", (unsigned long)s->in_l);
    b_raw(&b, ",\"in_r\":");
    b_fmt(&b, "%lu", (unsigned long)s->in_r);
    b_raw(&b, ",\"lvl_l\":");
    b_fmt(&b, "%d", s->lvl_l);
    b_raw(&b, ",\"lvl_r\":");
    b_fmt(&b, "%d", s->lvl_r);
    b_raw(&b, ",\"out_l\":");
    b_fmt(&b, "%lu", (unsigned long)s->out_l);
    b_raw(&b, ",\"out_r\":");
    b_fmt(&b, "%lu", (unsigned long)s->out_r);
    b_raw(&b, ",\"bad\":");
    b_fmt(&b, "%lu", (unsigned long)s->bad);
    if (s->have_tail) {
        b_raw(&b, ",\"snr_min\":");
        b_fmt(&b, "%.1f", (double)s->snr_min);
        b_raw(&b, ",\"snr_avg\":");
        b_fmt(&b, "%.1f", (double)s->snr_avg);
        b_raw(&b, ",\"snr_max\":");
        b_fmt(&b, "%.1f", (double)s->snr_max);
        b_raw(&b, ",\"bands_aggression\":");
        b_fmt(&b, "%lu", (unsigned long)s->bands_aggression);
        b_raw(&b, ",\"bands_bypassed\":");
        b_fmt(&b, "%lu", (unsigned long)s->bands_bypassed);
        b_raw(&b, ",\"gain\":");
        b_fmt(&b, "%.3f", (double)s->gain);
        b_raw(&b, ",\"mix\":");
        b_fmt(&b, "%.3f", (double)s->mix);
    }
    b_raw(&b, ",\"up\":");
    b_fmt(&b, "%llu", (unsigned long long)s->up);
    b_raw(&b, ",\"ver\":");
    b_str(&b, s->ver != NULL ? s->ver : "");
    b_raw(&b, ",\"sn\":");
    b_str(&b, s->sn != NULL ? s->sn : "");
    b_raw(&b, "}");
    return b.n;
}

size_t contract_boot_line(char *buf, size_t cap, const char *stage) {
    Builder b;
    b_init(&b, buf, cap);
    b_raw(&b, "{\"t\":\"boot\",\"stage\":");
    b_str(&b, stage != NULL ? stage : "");
    b_raw(&b, "}");
    return b.n;
}

size_t contract_crash_line(char *buf, size_t cap, const char *detail) {
    Builder b;
    b_init(&b, buf, cap);
    b_raw(&b, "{\"t\":\"crash\",\"detail\":");
    b_str(&b, detail != NULL ? detail : "");
    b_raw(&b, "}");
    return b.n;
}
