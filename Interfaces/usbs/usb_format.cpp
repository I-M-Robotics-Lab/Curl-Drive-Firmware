#include "usb_format.hpp"
#include <cstdint>
#include <cstdarg>
#include <cmath>

namespace usb {
namespace detail {

struct Out {
    char* dst;
    std::size_t cap;
    std::size_t len;
    bool truncated;
    inline void put(char c) {
        if (cap == 0) return;
        if (len < cap - 1) dst[len++] = c; else truncated = true;
    }
    inline void put_repeat(char c, int n) { while (n-- > 0) put(c); }
    inline void put_str(const char* s, std::size_t n) { for (std::size_t i=0;i<n;++i) put(s[i]); }
    inline void terminate() { if (cap == 0) return; if (truncated) dst[cap - 1] = '\0'; else dst[len] = '\0'; }
};

struct Fmt {
    bool left=false, plus=false, space=false, alt=false, zero=false;
    int width=-1, prec=-1;
    enum Len { NONE, L, LL } len=NONE;
    char conv='\0';
    bool upper=false;
};

inline bool is_digit(char c){ return c>='0'&&c<='9'; }

inline const char* parse_int(const char* p, int& out, int limit=1000000){
    int v=0; bool any=false;
    while(is_digit(*p)){ any=true; v=v*10+(*p-'0'); if(v>limit) v=limit; ++p; }
    out=any?v:-1; return p;
}

inline const char* parse_flags(const char* p, Fmt& f){
    for(;;){
        switch(*p){
            case '-': f.left=true; ++p; continue;
            case '+': f.plus=true; ++p; continue;
            case ' ': f.space=true; ++p; continue;
            case '#': f.alt=true; ++p; continue;
            case '0': f.zero=true; ++p; continue;
            default: return p;
        }
    }
}

inline const char* parse_len(const char* p, Fmt& f){
    if(*p=='l'){ ++p; if(*p=='l'){ ++p; f.len=Fmt::LL; } else { f.len=Fmt::L; } }
    return p;
}

template<typename U>
inline int utoa_base(U v, char* buf, int base, bool upper){
    const char* digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    int n=0;
    do { unsigned d = static_cast<unsigned>(v % base); buf[n++]=digits[d]; v = static_cast<U>(v / base); } while(v!=0);
    return n;
}

// ===== REPLACE THIS function in namespace usb::detail =====
template<typename S, typename U>
inline void format_int(Out& out,
                       const Fmt& f,
                       S sval,
                       U uval,
                       bool is_signed,
                       int base,
                       bool upper,
                       bool /*is_pointer*/ = false)
{
    int  width = (f.width < 0) ? 0 : f.width;
    int  prec  = (f.prec  < 0) ? -1 : f.prec;
    bool left  = f.left;
    bool zero  = f.zero && (prec < 0) && !left;   // '0' only when no precision and not left-justified
    bool plus  = f.plus;
    bool spc   = f.space && !plus;
    bool alt   = (base == 16) && f.alt;

    bool neg = false;
    U mag = uval;
    if (is_signed) {
        if (sval < 0) { neg = true; mag = (U)(- ( (typename std::make_signed<U>::type) sval )); }
        else { mag = (U)sval; }
    }

    char tmp[32];
    int nd = 0;
    if (mag == 0) {
        if (prec != 0) tmp[nd++] = '0';
    } else {
        if (base == 10) {
            while (mag && nd < (int)sizeof(tmp)) { U q = mag / 10; tmp[nd++] = char('0' + (mag - q*10)); mag = q; }
        } else { // base 16
            const char* digs = upper ? "0123456789ABCDEF" : "0123456789abcdef";
            while (mag && nd < (int)sizeof(tmp)) { unsigned v = (unsigned)(mag & 0xF); tmp[nd++] = digs[v]; mag >>= 4; }
        }
    }

    int min_digits = (prec < 0) ? 0 : prec;
    int zeros = (nd < min_digits) ? (min_digits - nd) : 0;

    char prefix[2];
    int npfx = 0;
    if (is_signed) {
        if (neg)      prefix[npfx++] = '-';
        else if (plus) prefix[npfx++] = '+';
        else if (spc)  prefix[npfx++] = ' ';
    }
    if (alt && nd > 0) {
        prefix[npfx++] = '0';
        prefix[npfx++] = upper ? 'X' : 'x';
    }

    int num_len = nd + zeros;
    int total   = npfx + num_len;
    int pad     = (width > total) ? (width - total) : 0;

    if (!left && !zero) {
        for (int i = 0; i < pad; ++i) out.put(' ');
        pad = 0;
    }

    for (int i = 0; i < npfx; ++i) out.put(prefix[i]);

    if (!left && zero) {
        for (int i = 0; i < pad; ++i) out.put('0');
        pad = 0;
    }

    for (int i = 0; i < zeros; ++i) out.put('0');

    for (int i = nd - 1; i >= 0; --i) out.put(tmp[i]);

    if (left && pad > 0) {
        for (int i = 0; i < pad; ++i) out.put(' ');
    }
}

inline void format_string(Out& out, const Fmt& f, const char* s){
    if(!s) s="(null)";
    std::size_t n=0; while(s[n]!='\0') ++n;
    if(f.prec>=0 && static_cast<std::size_t>(f.prec)<n) n=static_cast<std::size_t>(f.prec);
    int pad=(f.width>static_cast<int>(n))?(f.width-static_cast<int>(n)):0;
    if(!f.left) out.put_repeat(' ', pad);
    out.put_str(s, n);
    if(f.left) out.put_repeat(' ', pad);
}

inline void format_char(Out& out, const Fmt& f, char c){
    int pad=(f.width>1)?(f.width-1):0;
    if(!f.left) out.put_repeat(' ', pad);
    out.put(c);
    if(f.left) out.put_repeat(' ', pad);
}

inline void put_inf_nan(Out& out, const Fmt& f, bool neg, const char* word){
    char signch=0; if(neg) signch='-'; else if(f.plus) signch='+'; else if(f.space) signch=' ';
    int n=(signch?1:0)+3;
    int pad=(f.width>n)?(f.width-n):0;
    if(!f.left) out.put_repeat(' ', pad);
    if(signch) out.put(signch);
    out.put(word[0]); out.put(word[1]); out.put(word[2]);
    if(f.left) out.put_repeat(' ', pad);
}

inline double pow10i(int p) {
    double r = 1.0;
    if (p >= 0) { for (int i = 0; i <  p; ++i) r *= 10.0; }
    else        { for (int i = 0; i < -p; ++i) r /= 10.0; }
    return r;
}

inline void format_fixed(Out& out, Fmt f, double v){
    if(f.prec<0) f.prec=3;
    if(f.prec>6) f.prec=6;
    bool neg=std::signbit(v);
    if(std::isnan(v)){ put_inf_nan(out,f,false,"nan"); return; }
    if(std::isinf(v)){ put_inf_nan(out,f,neg,"inf"); return; }
    if(neg) v=-v;
    double scale=pow10i(f.prec);
    double rounded=std::floor(v*scale+0.5);
    unsigned long long int_part=static_cast<unsigned long long>(rounded/scale);
    unsigned long long frac_part=static_cast<unsigned long long>(std::llround(rounded - int_part*scale));
    char ibuf[32]; int in=utoa_base<unsigned long long>(int_part, ibuf, 10, false);
    char fbuf[16]; int fn=0;
    if(f.prec>0){
        for(int i=0;i<f.prec;++i){ fbuf[f.prec-1-i]=char('0'+(frac_part%10)); frac_part/=10; }
        fn=f.prec;
    }
    char signch=0; if(neg) signch='-'; else if(f.plus) signch='+'; else if(f.space) signch=' ';
    int core_len=(signch?1:0)+in+((f.prec>0||f.alt)?(1+fn):0);
    int pad=(f.width>core_len)?(f.width-core_len):0;
    if(!f.left && f.zero){ if(signch){ out.put(signch); signch=0; } out.put_repeat('0', pad); pad=0; }
    if(!f.left) out.put_repeat(' ', pad);
    if(signch) out.put(signch);
    for(int i=in-1;i>=0;--i) out.put(ibuf[i]);
    if(f.prec>0||f.alt){ out.put('.'); if(fn) out.put_str(fbuf, fn); }
    if(f.left) out.put_repeat(' ', pad);
}

}








// PARSERS ON, BUT NO format_int: only %s, %c, %d/%i/%u (32-bit), minimal handling.
// ===== REPLACE your current vformat with this (no %p) =====
std::size_t vformat(char* dst, std::size_t cap, const char* fmt, va_list ap){
    using namespace detail;
    Out out{dst,cap,0,false};
    if (cap == 0) return 0;
    if (!dst) return 0;
    if (!fmt) { dst[0] = '\0'; return 0; }

    const char* p = fmt;
    while (*p) {
        if (*p != '%') { out.put(*p++); continue; }
        ++p;
        if (*p == '\0') { out.put('%'); break; }
        if (*p == '%')  { out.put('%'); ++p; continue; }

        Fmt f{};
        f.width = -1;
        f.prec  = -1;

        p = parse_flags(p, f);
        if (is_digit(*p)) p = parse_int(p, f.width);
        if (*p == '.') { ++p; if (is_digit(*p)) p = parse_int(p, f.prec); else f.prec = 0; }
        p = parse_len(p, f);
        if (!(f.len == Fmt::NONE || f.len == Fmt::L || f.len == Fmt::LL)) f.len = Fmt::NONE;

        f.conv = *p ? *p++ : '\0';
        if (f.conv == 'X') f.upper = true;

        switch (f.conv) {
            case 'd':
            case 'i': {
                if (f.len == Fmt::LL) {
                    long long v = va_arg(ap, long long);
                    format_int<long long, unsigned long long>(out, f, v, (unsigned long long)v, true, 10, false);
                } else if (f.len == Fmt::L) {
                    long v = va_arg(ap, long);
                    format_int<long, unsigned long>(out, f, v, (unsigned long)v, true, 10, false);
                } else {
                    int v = va_arg(ap, int);
                    format_int<long long, unsigned long long>(out, f, (long long)v, (unsigned long long)(unsigned int)v, true, 10, false);
                }
            } break;

            case 'u': {
                if (f.len == Fmt::LL) {
                    unsigned long long v = va_arg(ap, unsigned long long);
                    format_int<long long, unsigned long long>(out, f, (long long)v, v, false, 10, false);
                } else if (f.len == Fmt::L) {
                    unsigned long v = va_arg(ap, unsigned long);
                    format_int<long, unsigned long>(out, f, (long)v, v, false, 10, false);
                } else {
                    unsigned int v = va_arg(ap, unsigned int);
                    format_int<long long, unsigned long long>(out, f, (long long)v, (unsigned long long)v, false, 10, false);
                }
            } break;

            case 'x':
            case 'X': {
                const bool up = (f.conv == 'X');
                if (f.len == Fmt::LL) {
                    unsigned long long v = va_arg(ap, unsigned long long);
                    format_int<long long, unsigned long long>(out, f, (long long)v, v, false, 16, up);
                } else if (f.len == Fmt::L) {
                    unsigned long v = va_arg(ap, unsigned long);
                    format_int<long, unsigned long>(out, f, (long)v, v, false, 16, up);
                } else {
                    unsigned int v = va_arg(ap, unsigned int);
                    format_int<long long, unsigned long long>(out, f, (long long)v, (unsigned long long)v, false, 16, up);
                }
            } break;

            case 'c': {
                int ch = va_arg(ap, int);
                format_char(out, f, (char)ch);
            } break;

            case 's': {
                const char* s = va_arg(ap, const char*);
                format_string(out, f, s);
            } break;

            case 'f': {
                double dv = va_arg(ap, double);
                format_fixed(out, f, dv);
            } break;

            // %p not supported: print literally
            case '\0':
            default: {
                out.put('%');
                if (f.conv) out.put(f.conv);
            } break;
        }
    }

    out.terminate();
    return out.truncated ? (cap ? cap - 1 : 0) : out.len;
}













std::size_t format(char* dst, std::size_t cap, const char* fmt, ...){
    va_list ap; va_start(ap, fmt);
    std::size_t n=vformat(dst,cap,fmt,ap);
    va_end(ap);
    return n;
}

}
