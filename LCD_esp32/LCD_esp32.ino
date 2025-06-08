#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

static LGFX lcd;
static LGFX_Sprite sprite(&lcd);

struct Button {
  int x, y, w, h;
  char label[6];
};

Button buttons[16];

String inputNumber = "";
int cursorPos = 0;
bool isEntered = false;
bool showFormatResult = false;
uint32_t formatResultTime = 0;

uint32_t lastBlink = 0;
bool cursorVisible = true;

// ======================= Hàm kiểm định định dạng =======================
bool isNumber(String s) {
  s.trim();
  if (s.length() == 0) return false;
  int start = (s[0] == '-') ? 1 : 0;
  for (int i = start; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

bool isValidCoordinateFormat(String str) {
  str.trim();
  if (str.length() == 0) return false;

  // Đếm số dấu phẩy trong chuỗi
  int commaCount = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str[i] == ',') commaCount++;
  }
  if (commaCount != 4) return false; // Cần đúng 4 dấu phẩy cho 2 tọa độ (x1,y1,z1-x2,y2,z2)

  // Tìm các vị trí dấu phẩy
  int comma1 = str.indexOf(',');
  int comma2 = str.indexOf(',', comma1 + 1);
  int comma3 = str.indexOf(',', comma2 + 1);
  int comma4 = str.indexOf(',', comma3 + 1);

  // Kiểm tra xem có đúng 4 dấu phẩy và dấu gạch giữa nằm giữa comma2 và comma3
  int dashIndex = str.indexOf('-', comma2 + 1);
  if (dashIndex <= comma2 || dashIndex >= comma3) return false; // Dấu - phải nằm giữa comma2 và comma3

  // Kiểm tra trường hợp hai dấu gạch giữa liên tục
  if (dashIndex + 1 < str.length() && str[dashIndex + 1] == '-') {
    // Nếu có hai dấu -, dấu - đầu tiên là dấu phân cách, bỏ qua dấu - thứ hai (thuộc số âm)
    dashIndex = str.indexOf('-'); // Lấy dấu - đầu tiên làm phân cách
  }

  // Chia chuỗi thành hai phần tại dashIndex
  String part1 = str.substring(0, dashIndex);
  String part2 = str.substring(dashIndex + 1);

  String parts[2] = {part1, part2};

  // Kiểm tra từng phần
  for (int i = 0; i < 2; i++) {
    int comma1 = parts[i].indexOf(',');
    if (comma1 == -1) return false;
    int comma2 = parts[i].indexOf(',', comma1 + 1);
    if (comma2 == -1) return false;
    if (parts[i].indexOf(',', comma2 + 1) != -1) return false; // Không được có dấu phẩy thừa

    String x = parts[i].substring(0, comma1);
    String y = parts[i].substring(comma1 + 1, comma2);
    String z = parts[i].substring(comma2 + 1);

    if (!isNumber(x) || !isNumber(y) || !isNumber(z)) return false;
  }

  return true;
}

// ======================= Giao diện =======================
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(24);
  lcd.startWrite();

  int btnWidth = lcd.width() / 4;
  int btnHeight = (lcd.height() * 3) / 4 / 4;
  int offsetX = 0;
  int offsetY = lcd.height() / 4;

  const char *labels[16] = {
    "1", "2", "3", "<",
    "4", "5", "6", ">",
    "7", "8", "9", "Del",
    "-", "0", ",", "Ent"
  };

  for (int i = 0; i < 16; i++) {
    int row = i / 4;
    int col = i % 4;
    buttons[i] = {
      offsetX + col * btnWidth + 5,
      offsetY + row * btnHeight + 5,
      btnWidth - 10,
      btnHeight - 10,
      {}
    };
    strncpy(buttons[i].label, labels[i], 5);
  }

  drawInterface();
}

// ======================= Vẽ giao diện =======================
void drawInterface() {
  lcd.fillScreen(lcd.color888(0, 0, 0));

  int inputBoxHeight = lcd.height() / 4 - 20;
  uint32_t boxColor;

  if (inputNumber.length() == 0) {
    boxColor = lcd.color888(0, 0, 255);  // Đang nhập
  } else if (showFormatResult) {
    if (isValidCoordinateFormat(inputNumber)) {
      boxColor = lcd.color888(0, 200, 0);  // Đúng định dạng
    } else {
      boxColor = lcd.color888(255, 0, 0);  // Sai định dạng
    }
  } else {
    boxColor = lcd.color888(0, 0, 255);  // Mặc định
  }

  lcd.fillRect(10, 10, lcd.width() - 20, inputBoxHeight, boxColor);
  lcd.drawRect(10, 10, lcd.width() - 20, inputBoxHeight, lcd.color888(255, 255, 255));

  lcd.setTextColor(lcd.color888(255, 255, 255));
  lcd.setTextSize(2);
  lcd.setCursor(20, 12 + inputBoxHeight / 2 - 10);
  lcd.print(inputNumber);

  if (cursorVisible && !showFormatResult) {
    int cursorX = 20 + cursorPos * 12;
    lcd.drawLine(cursorX, 10 + inputBoxHeight / 2 - 10, cursorX, 10 + inputBoxHeight / 2 + 10, lcd.color888(255, 255, 255));
  }

  for (int i = 0; i < 16; i++) {
    lcd.fillRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, lcd.color888(80, 80, 80));
    lcd.drawRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, lcd.color888(255, 255, 255));
    lcd.setTextColor(lcd.color888(255, 255, 255));
    lcd.setTextSize((strcmp(buttons[i].label, "Del") == 0 || strcmp(buttons[i].label, "Ent") == 0) ? 1 : 2);
    lcd.setCursor(buttons[i].x + buttons[i].w / 2 - 10, buttons[i].y + buttons[i].h / 2 - 10);
    lcd.print(buttons[i].label);
  }
}

// ======================= Vòng lặp chính =======================
void loop() {
  uint16_t x, y;
  static uint32_t lastTouch = 0;

  // Nháy con trỏ
  if (!showFormatResult && millis() - lastBlink > 500) {
    cursorVisible = !cursorVisible;
    lastBlink = millis();
    drawInterface();
  }

  // Kết thúc hiệu ứng kết quả sau 3 giây
  if (showFormatResult && millis() - formatResultTime > 3000) {
    showFormatResult = false;
    isEntered = false;
    drawInterface();
  }

  if (lcd.getTouch(&x, &y) && (millis() - lastTouch > 200)) {
    lastTouch = millis();
    for (int i = 0; i < 16; i++) {
      if (x >= buttons[i].x && x < buttons[i].x + buttons[i].w &&
          y >= buttons[i].y && y < buttons[i].y + buttons[i].h) {

        String lbl = buttons[i].label;

        if (lbl == "Del") {
          if (cursorPos > 0) {
            inputNumber.remove(cursorPos - 1, 1);
            cursorPos--;
          }
          isEntered = false;
        } else if (lbl == "<") {
          if (cursorPos > 0) cursorPos--;
        } else if (lbl == ">") {
          if (cursorPos < inputNumber.length()) cursorPos++;
        } else if (lbl == "Ent") {
          isEntered = true;
          showFormatResult = true;
          formatResultTime = millis();

          if (isValidCoordinateFormat(inputNumber)) {
            Serial.println(inputNumber);
          } else {
            Serial.println("Invalid format!");
          }

        } else {
          inputNumber = inputNumber.substring(0, cursorPos) + lbl + inputNumber.substring(cursorPos);
          cursorPos += lbl.length();
          isEntered = false;
        }

        // Hiệu ứng nhấn nút
        lcd.fillRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, lcd.color888(255, 0, 0));
        delay(80);
        drawInterface();
        break;
      }
    }
  }
}