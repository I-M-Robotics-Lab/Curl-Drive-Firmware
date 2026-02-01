// usb.cpp
#include "usb.hpp"
#include <cstring>
#include <cmath>

extern "C" {
  #include "usbd_cdc_if.h"
  extern USBD_HandleTypeDef hUsbDeviceFS;
}

namespace {
constexpr std::size_t RX_RING_SIZE = 1024;
constexpr std::size_t LINE_MAX     = 256;
constexpr uint16_t    USB_MPS      = 64;

volatile uint8_t     rx_ring[RX_RING_SIZE];
volatile std::size_t rx_head = 0;
volatile std::size_t rx_tail = 0;

inline std::size_t rb_count() { return (rx_head + RX_RING_SIZE - rx_tail) % RX_RING_SIZE; }
inline std::size_t rb_free()  { return (rx_tail + RX_RING_SIZE - rx_head - 1) % RX_RING_SIZE; }

char         line_buf[LINE_MAX];
std::size_t  line_len = 0;

uint8_t tx_scratch[USB_MPS];
}




std::size_t usb::write(const uint8_t* data, std::size_t len)
{
  if (!data || len == 0) return 0;
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return 0;

  std::size_t sent = 0;
  while (len) {
    const uint16_t chunk = static_cast<uint16_t>(len > USB_MPS ? USB_MPS : len);
    std::memcpy(tx_scratch, data, chunk);
    const int8_t st = CDC_Transmit_FS(tx_scratch, chunk);
    if (st != static_cast<int8_t>(USBD_OK)) break;
    data += chunk; len -= chunk; sent += chunk;
  }
  return sent;
}

std::size_t usb::write(const char* cstr)
{
  if (!cstr) return 0;
  return usb::write(reinterpret_cast<const uint8_t*>(cstr), std::strlen(cstr));
}
/*
bool usb::writeLine(const char* cstr)
{
  if (!cstr) return false;
  const std::size_t n = std::strlen(cstr);
  if (usb::write(reinterpret_cast<const uint8_t*>(cstr), n) != n) return false;
  static const uint8_t crlf[2] = {'\r','\n'};
  return usb::write(crlf, 2) == 2;
}
*/
bool usb::writeLine(const char* cstr)
{
  if (!cstr) return false;
  const std::size_t n = std::strlen(cstr);

  static char combined[512];
  if (n >= sizeof(combined) - 2) return false;

  std::memcpy(combined, cstr, n);
  combined[n] = '\r';
  combined[n + 1] = '\n';

  return usb::write(reinterpret_cast<const uint8_t*>(combined), n + 2) == (n + 2);
}

std::size_t usb::available()
{
  return rb_count();
}

std::size_t usb::read(uint8_t* out, std::size_t max)
{
  if (!out || max == 0) return 0;
  std::size_t n = 0;
  while (n < max && rx_tail != rx_head) {
    out[n++] = rx_ring[rx_tail];
    rx_tail  = (rx_tail + 1) % RX_RING_SIZE;
  }
  return n;
}

bool usb::readByte(uint8_t& out)
{
  if (rx_tail == rx_head) return false;
  out = rx_ring[rx_tail];
  rx_tail = (rx_tail + 1) % RX_RING_SIZE;
  return true;
}

bool usb::readLine(char* out, std::size_t max)
{
  if (!out || max == 0) return false;

  while (rx_tail != rx_head) {
    uint8_t b = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % RX_RING_SIZE;

    if (b == '\r') b = '\n';
    if (b == '\n') {
      const std::size_t to_copy = (line_len < (max - 1)) ? line_len : (max - 1);
      std::memcpy(out, line_buf, to_copy);
      out[to_copy] = '\0';
      line_len = 0;
      return true;
    }
    if (line_len < (LINE_MAX - 1)) line_buf[line_len++] = static_cast<char>(b);
  }
  return false;
}






namespace {
struct Out {
  char* p;
  std::size_t cap;
  std::size_t n;
  bool trunc;
  inline void put(char c){ if(cap && n+1<cap) p[n++]=c; else trunc=true; }
  inline void puts(const char* s){ if(!s) s="(null)"; while(*s){ if(cap && n+1<cap) p[n++]=*s++; else {trunc=true; break;} } }
  inline void term(){ if(!cap) return; if(n<cap) p[n]='\0'; else p[cap-1]='\0'; }
};

inline void append(Out& out, const char* s){ out.puts(s); }
inline void append(Out& out, char* s){ out.puts(s); }
inline void append(Out& out, char c){ out.put(c); }
inline void append(Out& out, bool v){ out.puts(v?"true":"false"); }

inline void utoa_dec(unsigned long long v, char* buf, int& len){
  char tmp[32]; int n=0;
  do { unsigned long long q=v/10ULL; unsigned r=(unsigned)(v-q*10ULL); tmp[n++]=char('0'+r); v=q; } while(v && n<32);
  for(int i=n-1;i>=0;--i) buf[len++]=tmp[i];
}

template<class T>
inline typename std::enable_if<
  std::is_integral<T>::value &&
  !std::is_same<T,bool>::value &&
  !std::is_same<T,char>::value
, void>::type
append(Out& out, T v){
  char buf[32]; int len=0;
  if(std::is_signed<T>::value && v<0){
    out.put('-');
    unsigned long long mag=(unsigned long long)(-(long long)v);
    utoa_dec(mag,buf,len);
  }else{
    unsigned long long mag=(unsigned long long)(v);
    utoa_dec(mag,buf,len);
  }
  buf[len]='\0'; out.puts(buf);
}

inline void append(Out& out, float v) {
  int32_t iv = (v >= 0.0f) ? (int32_t)(v*10000.0f + 0.5f) : (int32_t)(v*10000.0f - 0.5f);
  if (iv < 0) { out.put('-'); iv = -iv; }
  uint32_t I = (uint32_t)(iv / 10000), F = (uint32_t)(iv % 10000);
  char t[16]; int n = 0; uint32_t x = I;
  do { t[n++] = char('0' + (x % 10)); x /= 10; } while (x && n < 16);
  for (int i = n - 1; i >= 0; --i) out.put(t[i]);
  out.put('.');
  out.put(char('0' + (F/1000)%10));
  out.put(char('0' + (F/100)%10));
  out.put(char('0' + (F/10)%10));
  out.put(char('0' + (F%10)));
}
inline void append(Out& out, double v) { append(out, (float)v); }

inline void append(Out& out, const void* ptr){
  if(!ptr){ out.puts("(null)"); return; }
  out.puts("0x");
  uintptr_t v=reinterpret_cast<uintptr_t>(ptr);
  char rev[2+sizeof(uintptr_t)*2]; int n=0;
  if(v==0){ rev[n++]='0'; }
  else { const char* d="0123456789abcdef"; while(v && n<(int)sizeof(rev)){ rev[n++]=d[v&0xF]; v>>=4; } }
  while(n--) out.put(rev[n]);
}

inline void build(Out&){}

template<class First, class... Rest>
inline void build(Out& out, const First& f, const Rest&... r){
  append(out,f); build(out,r...);
}

}





// for usbd_cdc_if.c
extern "C" void usb_on_rx_c(const uint8_t* data, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i) {
    if (rb_free() == 0) break;
    rx_ring[rx_head] = data[i];
    rx_head = (rx_head + 1) % RX_RING_SIZE;
  }
}


