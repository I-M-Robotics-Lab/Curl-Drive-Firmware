// usb.hpp
#pragma once
#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include "usb_format.hpp"

namespace usb {

std::size_t write(const uint8_t* data, std::size_t len);
std::size_t write(const char* cstr);
bool        writeLine(const char* cstr);

std::size_t available();
std::size_t read(uint8_t* out, std::size_t max);
bool        readByte(uint8_t& out);
bool        readLine(char* out, std::size_t max);


namespace txt {
struct Out{ char* p; std::size_t cap; std::size_t n; bool trunc;
  inline void put(char c){ if(cap && n+1<cap) p[n++]=c; else trunc=true; }
  inline void puts(const char* s){ if(!s) s="(null)"; while(*s){ if(cap && n+1<cap) p[n++]=*s++; else{trunc=true; break;} } }
  inline void term(){ if(!cap) return; if(n<cap) p[n]='\0'; else p[cap-1]='\0'; }
};
inline void utoa_dec(unsigned long long v, char* buf, int& len){ char t[32]; int k=0; do{ unsigned long long q=v/10ULL; unsigned r=(unsigned)(v-q*10ULL); t[k++]=char('0'+r); v=q; }while(v&&k<32); for(int i=k-1;i>=0;--i) buf[len++]=t[i]; }
inline void append(Out& o,const char* s){ o.puts(s); }
inline void append(Out& o,char* s){ o.puts(s); }
inline void append(Out& o,char c){ o.put(c); }
inline void append(Out& o,bool v){ o.puts(v?"true":"false"); }
template<class T>
inline typename std::enable_if<std::is_integral<T>::value && !std::is_same<T,bool>::value && !std::is_same<T,char>::value, void>::type
append(Out& o,T v){ char b[32]; int L=0; if(std::is_signed<T>::value && v<0){ o.put('-'); unsigned long long m=(unsigned long long)(-(long long)v); utoa_dec(m,b,L); } else { unsigned long long m=(unsigned long long)v; utoa_dec(m,b,L); } b[L]='\0'; o.puts(b); }

inline void append(Out& o, float v) {
  int32_t iv = (v >= 0.0f) ? (int32_t)(v*10000.0f + 0.5f) : (int32_t)(v*10000.0f - 0.5f);
  if (iv < 0) { o.put('-'); iv = -iv; }
  uint32_t I = (uint32_t)(iv / 10000), F = (uint32_t)(iv % 10000);
  char t[16]; int n = 0; uint32_t x = I;
  do { t[n++] = char('0' + (x % 10)); x /= 10; } while (x && n < 16);
  for (int i = n - 1; i >= 0; --i) o.put(t[i]);
  o.put('.');
  o.put(char('0' + (F/1000)%10));
  o.put(char('0' + (F/100)%10));
  o.put(char('0' + (F/10)%10));
  o.put(char('0' + (F%10)));
}
inline void append(Out& o, double v) { append(o, (float)v); }


inline void append(Out& o,const void* p){ if(!p){ o.puts("(null)"); return;} o.puts("0x"); uintptr_t x=reinterpret_cast<uintptr_t>(p); char r[2+sizeof(uintptr_t)*2]; int n=0; if(x==0){ r[n++]='0'; } else { const char* d="0123456789abcdef"; while(x&&n<(int)sizeof(r)){ r[n++]=d[x&0xF]; x>>=4; } } while(n--) o.put(r[n]); }
inline void build(Out&){}
template<class F,class... R>
inline void build(Out& o,const F& f,const R&... r){ append(o,f); build(o,r...); }
}

template<class... Ts>
inline std::size_t print(const Ts&... xs){
  static char buf[384];
  txt::Out out{buf,sizeof(buf),0,false};
  txt::build(out,xs...);
  out.term();
  return usb::write(buf);
}

template<class... Ts>
inline std::size_t println(const Ts&... xs){
  static char buf[384];
  txt::Out out{buf,sizeof(buf),0,false};
  txt::build(out,xs...);
  out.term();
  return usb::writeLine(buf) ? (out.n+2) : 0;
}


}





/* ============================================================================
 *
 * usb::write(const uint8_t* ptr, std::size_t len)
 *   ptr: pointer to byte buffer to send
 *   len: number of bytes at ptr
 *   return: std::size_t bytes accepted (<=len)
 *
 * usb::write(const char* cstr)
 *   cstr: C-string to send (no newline)
 *   return: std::size_t bytes accepted
 *
 * usb::writeLine(const char* cstr)
 *   cstr: C-string payload
 *   return: bool true if full "cstr\\r\\n" accepted
 *
 * usb::available()
 *   return: std::size_t bytes currently buffered (RX)
 *
 * usb::read(uint8_t* out, std::size_t max)
 *   out: destination buffer
 *   max: max bytes to copy
 *   return: std::size_t bytes copied
 *
 * usb::readByte(uint8_t& out)
 *   out: reference to receive one byte
 *   return: bool true if a byte was read
 *
 * usb::readLine(char* out, std::size_t max)
 *   out: destination for text line (NUL-terminated)
 *   max: buffer size (writes at most max-1 chars)
 *   return: bool true if a full line was produced
 *
 * ========================================================================== */












