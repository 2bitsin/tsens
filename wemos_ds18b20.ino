#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <functional>
#include <map>
#include <string>
#include <chrono>
#include <cstdint>

struct any
{
  template<typename... T> 
  any(T&&...) {}
};

////////////////////////////////////////////////////////////////
///   CONFIG
////////////////////////////////////////////////////////////////

const static std::uint32_t G_magic = 'SENS';
const static std::uint32_t G_sensor_id = 2;
const static std::uint32_t G_DS18B20 = 0x0D518B20;

const static char G_prefix[] = "KS10G30";
const static char G_password[] = "dominelis";
const static char G_otapwd[] = "LimitBreak1024";

const static unsigned short G_tempport = 10000;
const static auto G_tick = std::chrono::seconds(1);

////////////////////////////////////////////////////////////////
///     VARIABLES
////////////////////////////////////////////////////////////////

OneWire ow(5);
DallasTemperature ds(&ow);
ArduinoOTAClass ota;

static char G_hostname[128] = "";
static uint32_t G_address[2] = {G_sensor_id, 0};
int retries_ = 0;
any (*loop_) () = nullptr; 

////////////////////////////////////////////////////////////////
///     CODE
////////////////////////////////////////////////////////////////

any wait();
any scan();
any exec();

void setup_services()
{
  MDNS.begin(G_hostname, WiFi.localIP());  
  ota.begin();  
  ota.setPassword(G_otapwd);
  ota.setHostname(G_hostname);
  ota.setPort(G_tempport-1);
}

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  ds.begin();    
  if (!ds.getAddress((uint8_t*)&G_address[0], 0))
  {
    G_address[0] = G_sensor_id;
    G_address[1] = 0;
    Serial.println("Failed to identify sensor.");
  }
  sprintf(G_hostname, "node_%08x%08x", G_address[1], G_address[0]);
  WiFi.hostname(G_hostname);
  loop_ = &scan;
}

void wait_until(std::chrono::steady_clock::time_point t)
{
  using namespace std::chrono;
  auto dt = duration_cast<milliseconds>(t - steady_clock::now());
  if (dt.count() <= 0)
    return;
  delay(dt.count());
}

void send(int32_t temp)
{
  std::uint32_t buffer [] = 
  {
    G_magic,
    G_DS18B20,
    G_address[0],
    G_address[1],
    reinterpret_cast<const int32_t&>(temp)
  };

  WiFiUDP udp;
  IPAddress ip(255, 255, 255, 255);
  udp.beginPacket(ip, G_tempport);
  udp.write((const char*)&buffer, sizeof(buffer));
  udp.endPacket();

  udp.beginPacket(ip, G_tempport + 1);
  udp.printf(R"({
    "magic": "SENS", 
    "sensor_type": "DS18B20", 
    "sensor_id": "%08X-%08X", 
    "value": "%f"
  })", G_address[1], G_address[0], temp);
  udp.endPacket();
}

any scan ()
{
  std::multimap<long, int> sorted;
  Serial.println("Scanning networks...");
  auto n = WiFi.scanNetworks();
  for (auto i = 0; i < n; ++i)
  {
    auto ssid = WiFi.SSID(i);
    auto rssi = WiFi.RSSI(i);
    Serial.printf("%s (%d dBm)\n", ssid.c_str(), rssi);
    if (!ssid.startsWith(G_prefix))
      continue;
    sorted.emplace(rssi, i);
  }
  if (sorted.size() > 0)
  {
    auto index = sorted.rbegin()->second;
    auto ssid = WiFi.SSID(index);
    Serial.printf("Attempting connection to : %s ...\n", ssid.c_str());       
    WiFi.begin(ssid.c_str(), G_password);
    retries_ = 10;
    return loop_ = &wait;
  }  
  return loop_;
}

any wait()
{
  Serial.printf("(Re)trying %d ...\n", retries_);  
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("Connected, hostname: %s\n", G_hostname);
    setup_services();
    return loop_ = &exec;    
  }
  if (--retries_ < 1)
    return loop_ = &scan;
  return loop_;
}

any exec()
{
  const static R = 1.0f/128.0f;
  uint8_t buff[16];
  ds.requestTemperatures();
  ds.getAddress(buff, 0);
  auto t = ds.getTemp(buff);
  send(t);
  Serial.printf("DS18B20; %08X-%08X; %f\n", G_address[1], G_address[0], t * R);
}

void loop () 
{
  using namespace std::chrono;
  auto next_ = steady_clock::now() + G_tick;
  loop_();  
  yield();  
  while(steady_clock::now() < next_)
  { 
    ota.handle();
    MDNS.update();
    yield();
  }
}
