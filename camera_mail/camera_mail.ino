#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "camera_pins.h"

#define ENABLE_SMTP  // Allow SMTP class and data
#define ENABLE_DEBUG // Allow debugging
#define READYMAIL_DEBUG_PORT Serial
#include "ReadyMail.h"

/** The smtp host name e.g. smtp.gmail.com for GMail or smtp.office365.com for Outlook or smtp.mail.yahoo.com */
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

/* The sign in credentials */
#define AUTHOR_EMAIL "ee459team7@gmail.com"
#define AUTHOR_PASSWORD "velwapotrpbskhaz"

/* Recipient's email*/
#define RECIPIENT_EMAIL "luok@usc.edu"
#define DOMAIN_OR_IP "127.0.0.1"

#define SSL_MODE true
#define AUTHENTICATION true

#define ALERT_PIN 13

bool email_sent = false;

WiFiClientSecure ssl_client;
SMTPClient smtp(ssl_client);


const char *ssid_Router     = "kevluo";  //input your wifi name
const char *password_Router = "kevinjhluo";  //input your wifi passwords
camera_config_t config;

void startCameraServer();
void camera_init();

void smtpCb(SMTPStatus status)
{
    if (status.progressUpdated)
        ReadyMail.printf("ReadyMail[smtp][%d] Uploading file %s, %d %% completed\n", status.state, status.filename.c_str(), status.progress);
    else
        ReadyMail.printf("ReadyMail[smtp][%d]%s\n", status.state, status.text.c_str());
    // The status.state is the smtp_state enum defined in src/smtp/Common.h
}

void setMessageTime(SMTPMessage &msg, const String &date, uint32_t timestamp = 0)
{
    // Three methods to set the message date.
    // Can be selected one of these methods.
    if (date.length())
    {
        msg.addHeader("Date: " + date);
        msg.date = date;
    }
    if (timestamp > 0) // The UNIX timestamp (seconds since Midnight Jan 1, 1970)
        msg.timestamp = timestamp;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  pinMode(ALERT_PIN, INPUT);
  delay (2000);

  
  camera_init();

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 0);        //1-Upside down, 0-No operation
  s->set_hmirror(s, 0);      //1-Reverse left and right, 0-No operation
  s->set_brightness(s, 1);   //up the blightness just a bit
  s->set_saturation(s, -1);  //lower the saturation

  WiFi.begin(ssid_Router, password_Router);
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  delay(1000);

  startCameraServer();

  delay(500);

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  ssl_client.setInsecure();
   smtp.connect(SMTP_HOST, SMTP_PORT, DOMAIN_OR_IP, smtpCb, SSL_MODE);
    if (!smtp.isConnected())
        return;

    if (AUTHENTICATION)
    {
        smtp.authenticate(AUTHOR_EMAIL, AUTHOR_PASSWORD, readymail_auth_password);
        if (!smtp.isAuthenticated())
            return;
    }

  
}

void loop() {
  int alertSignal = digitalRead(ALERT_PIN);

  if(!email_sent){
    if (alertSignal == HIGH) {
      
      SMTPMessage msg;
      msg.sender.name = "Porch Protector";
      msg.sender.email = AUTHOR_EMAIL;
      msg.subject = "Security Alert";
      msg.addRecipient("User", RECIPIENT_EMAIL);
  
      String ip = WiFi.localIP().toString();
      String bodyText = "Irregular activity detected, click the link below for live video feed \n";
      bodyText += "http://"+ip;
  
      msg.text.content = bodyText;
      msg.text.transfer_encoding = "base64";
  
  
      setMessageTime(msg, "Fri, 14 Mar 2025 09:13:23 +0300");
  
      smtp.send(msg);
      Serial.println("alert email sent");
      email_sent = true;
   
    }else{
      Serial.println("waiting for alert pin"); 
    }
  }

  delay(1000);
}

void camera_init() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 2;
}
