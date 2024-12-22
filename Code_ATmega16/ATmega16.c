#define F_CPU 8000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/eeprom.h>    // Для работы с EEPROM
#include <avr/pgmspace.h>  // Для работы с PROGMEM
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/wdt.h>         // Для Watchdog Timer

#define TW_WRITE 0 // Запись по I2C
#define NUM_SLAVES 112

// ===== Глобальные переменные =====
uint8_t slaveAddresses[NUM_SLAVES];
uint8_t slaveEnabled[NUM_SLAVES];
uint8_t currentSlave = 0;

// Сигнатура для проверки данных в EEPROM
const uint8_t eepromSignature[4] = {0xDE, 0xAD, 0xBE, 0xEF};

// Адреса в EEPROM
uint8_t EEMEM eeSignature[4];
uint8_t EEMEM eeSlaveAddresses[NUM_SLAVES];
uint8_t EEMEM eeSlaveEnabled[NUM_SLAVES];
uint8_t EEMEM eeCurrentSlave;

// Строки в PROGMEM для экономии SRAM
const char str_welcome[] PROGMEM             = "This is AVR ATmega16 with Round Robin";
const char str_data_save[] PROGMEM           = "Data saved to EEPROM.";
const char str_data_clear[] PROGMEM          = "EEPROM cleared. After next reset defaults will be used.";
const char str_reset[] PROGMEM               = "System is resetting...";
const char str_err_inv_num[] PROGMEM         = "Error: Invalid slave number";
const char str_err_no_active[] PROGMEM       = "Error: No active slaves available";
const char str_err_current_disable[] PROGMEM = "Error: Current slave is disabled";
const char str_help_header[] PROGMEM  = "Available commands:";
const char str_help_print1[] PROGMEM  = "  * print [N]      - Display all active slave addresses or the next N slaves.";
const char str_help_print2[] PROGMEM  = "               Without N, displays all active addresses.";
const char str_help_enable[] PROGMEM  = "  * enable N    - Enable slave with number N.";
const char str_help_disable[] PROGMEM = "  * disable N   - Disable slave with number N.";
const char str_help_current[] PROGMEM = "  * current      - Show the current active slave address.";
const char str_help_save[] PROGMEM    = "  * save          - Save current config to EEPROM.";
const char str_help_clear[] PROGMEM   = "  * clear          - Clear EEPROM data and reset configuration.";
const char str_help_reset[] PROGMEM   = "  * reset          - Programmatically reset the microcontroller.";
const char str_help_help[] PROGMEM    = "  * help           - Show this help message.";
const char str_help_note[] PROGMEM    = "Note:";
const char str_help_note2[] PROGMEM   = "  By default, any entered string will be treated as a message to";
const char str_help_note3[] PROGMEM   = "  send to the current slave device. The system will balance the";
const char str_help_note4[] PROGMEM   = "  load using the Round Robin algorithm.";

// ===== UART Функции =====
void USARTInit(unsigned int ubrr) {
    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)(ubrr);
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
}

void USARTTransmitChar(char c) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = c;
}

void USARTTransmitString(const char *str) {
    while (*str) USARTTransmitChar(*str++);
}

void USARTTransmitStringLn(const char *str) {
    USARTTransmitString(str);
    USARTTransmitChar('\r');
    USARTTransmitChar('\n');
}

// Вывод строки из PROGMEM
void USARTTransmitStringP(const char *pstr) {
    char c;
    while ((c = pgm_read_byte(pstr++))) USARTTransmitChar(c);
}

void USARTTransmitStringLnP(const char *pstr) {
    USARTTransmitStringP(pstr);
    USARTTransmitChar('\r');
    USARTTransmitChar('\n');
}

void USARTReceiveString(char *buf, uint8_t n) {
    uint8_t idx = 0;
    char c;
    do {
        while (!(UCSRA & (1 << RXC)));
        c = UDR;
        buf[idx++] = c;
    } while ((idx < n - 1) && (c != '\r'));
    buf[idx - 1] = '\0';
}

// ===== I2C (TWI) =====
#define SCL_CLOCK 100000L
void TWIInit(void) {
    TWSR = 0x00;
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
    TWCR = (1 << TWEN);
}

void TWIStart(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void TWIStop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while (TWCR & (1 << TWSTO));
}

void TWIWrite(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

// ===== LCD =====
#define LCD_RS PC2
#define LCD_RW PC3
#define LCD_E  PC4

#define LCD_DATA_PORT PORTA
#define LCD_DATA_DDR  DDRA

void LCD_Command(unsigned char cmd) {
    LCD_DATA_PORT = cmd;
    PORTC &= ~(1 << LCD_RS);
    PORTC |= (1 << LCD_E);
    _delay_us(1);
    PORTC &= ~(1 << LCD_E);
    _delay_ms(2);
}

void LCD_Char(unsigned char data) {
    LCD_DATA_PORT = data;
    PORTC |= (1 << LCD_RS);
    PORTC |= (1 << LCD_E);
    _delay_us(1);
    PORTC &= ~(1 << LCD_E);
    _delay_ms(2);
}

void LCD_Init() {
    LCD_DATA_DDR = 0xFF;
    DDRC |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_E);
    _delay_ms(20);

    LCD_Command(0x38);
    LCD_Command(0x0C);
    LCD_Command(0x06);
    LCD_Command(0x01);
    _delay_ms(2);
}

void LCD_Clear() {
    LCD_Command(0x01);
    _delay_ms(2);
}

void LCD_SetCursor(unsigned char row, unsigned char col) {
    static const uint8_t pos[4] PROGMEM = {0x80, 0xC0, 0x94, 0xD4};
    uint8_t base = pgm_read_byte(&pos[row - 1]);
    LCD_Command(base + col - 1);
}

void LCD_String(char *str) {
    uint8_t i = 0;
    while (str[i]) LCD_Char(str[i++]);
}

// ===== Функции инициализации и EEPROM =====

void initSlaveAddresses() {
    for (uint8_t i = 0; i < NUM_SLAVES; i++) {
        slaveAddresses[i] = 0x08 + i;
        slaveEnabled[i] = 1;
    }

    currentSlave = 0;
}

void toBinary(uint8_t value, char *binaryStr) {
    for (int i = 7; i >= 0; i--) binaryStr[7 - i] = (value & (1 << i)) ? '1' : '0';
    binaryStr[8] = '\0';
}

// Сохранение данных в EEPROM
void saveDataToEEPROM() {
    // Сохранение сигнатуры
    for (uint8_t i = 0; i < 4; i++)
        eeprom_update_byte(&eeSignature[i], eepromSignature[i]);

    // Сохранение адресов
    for (uint8_t i = 0; i < NUM_SLAVES; i++)
        eeprom_update_byte(&eeSlaveAddresses[i], slaveAddresses[i]);

    // Сохранение статуса
    for (uint8_t i = 0; i < NUM_SLAVES; i++)
        eeprom_update_byte(&eeSlaveEnabled[i], slaveEnabled[i]);

    // Сохранение текущего слейва
    eeprom_update_byte(&eeCurrentSlave, currentSlave);

    USARTTransmitStringLnP(str_data_save);
}

// Загрузка данных из EEPROM
uint8_t loadDataFromEEPROM() {
    // Проверка сигнатуры
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t b = eeprom_read_byte(&eeSignature[i]);
        if (b != eepromSignature[i]) {
            return 0; // Сигнатура не совпала
        }
    }

    // Загрузка адресов
    for (uint8_t i = 0; i < NUM_SLAVES; i++)
        slaveAddresses[i] = eeprom_read_byte(&eeSlaveAddresses[i]);

    // Загрузка статуса
    for (uint8_t i = 0; i < NUM_SLAVES; i++)
        slaveEnabled[i] = eeprom_read_byte(&eeSlaveEnabled[i]);

    // Загрузка текущего слейва
    currentSlave = eeprom_read_byte(&eeCurrentSlave);

    return 1;
}

// Инициализация при старте
void initialLoad() {
    if (!loadDataFromEEPROM()) {
        // Если данные ещё не сохранены, создаём по умолчанию
        initSlaveAddresses();
        // Сохраняем их
        saveDataToEEPROM();
    }
}

// ===== Управление слейвами =====

void enableSlave(uint8_t slaveNumber) {
    if (slaveNumber < 1 || slaveNumber > NUM_SLAVES) {
        USARTTransmitStringLnP(str_err_inv_num);
        return;
    }
    slaveEnabled[slaveNumber - 1] = 1;
    char outBuffer[30];
    sprintf(outBuffer, "Slave %3d enabled", slaveNumber);
    USARTTransmitStringLn(outBuffer);
}

void disableSlave(uint8_t slaveNumber) {
    if (slaveNumber < 1 || slaveNumber > NUM_SLAVES) {
        USARTTransmitStringLnP(str_err_inv_num);
        return;
    }
    slaveEnabled[slaveNumber - 1] = 0;
    char outBuffer[30];
    sprintf(outBuffer, "Slave %3d disabled", slaveNumber);
    USARTTransmitStringLn(outBuffer);
}

// ===== Функция для программного сброса =====
void softwareReset() {
    // Включаем Watchdog Timer с минимальным тайм-аутом
    wdt_enable(WDTO_15MS);
    // Бесконечный цикл ожидания сброса
    while(1) {}
}

// ===== Функция для отключения Watchdog Timer =====
void disableWatchdog() {
    // Отключаем Watchdog Timer согласно последовательности из даташита
    WDTCR |= (1 << WDE) | (1 << WDTOE); // Установить WDE и WDTOE
    WDTCR = 0x00;                      // Отключить WDT
}

int main(void) {
    char buffer[20];
    char outBuffer[50];
    char binaryAddress[9];
    char displayBuffer[21];

    // Отключаем Watchdog Timer при старте, чтобы избежать непреднамеренных сбросов
    MCUCSR |= (1 << WDRF);   // Очищаем флаг сброса от WDT
    disableWatchdog();       // Отключаем Watchdog Timer

    USARTInit(MYUBRR);
    TWIInit();
    LCD_Init();

    initialLoad(); // Загружаем данные из EEPROM или создаём новые
    
    USARTTransmitStringLnP(str_welcome);

    while (1) {
        USARTReceiveString(buffer, sizeof(buffer));
        sprintf(outBuffer, "> %s", buffer);
        USARTTransmitStringLn(outBuffer);

        if (strncmp(buffer, "print", 5) == 0) {
            uint8_t n = 0;
            if (strlen(buffer) > 6) {
                n = atoi(&buffer[6]);
            }
            
            uint8_t index = 0;
            if (n > 0 && n < NUM_SLAVES) {
                index = currentSlave;
            }

            uint8_t printed = 0;
            for (uint8_t i = 0; i < NUM_SLAVES && (n == 0 || printed < n); i++) {
                toBinary(slaveAddresses[index], binaryAddress);
                const char *status = (slaveEnabled[index]) ? "Enabled" : "Disabled";
                if (index == currentSlave) {
                    sprintf(outBuffer, "* Slave %3d: 0x%02X (Binary: %s) [%s]",
                        index + 1, slaveAddresses[index], binaryAddress, status);
                }
                else {
                    sprintf(outBuffer, "Slave %3d: 0x%02X (Binary: %s) [%s]",
                        index + 1, slaveAddresses[index], binaryAddress, status);
                }
                USARTTransmitStringLn(outBuffer);
                printed++;
                index = (index + 1) % NUM_SLAVES;
            }
        } else if (strncmp(buffer, "enable ", 7) == 0) {
            uint8_t slaveNumber = atoi(&buffer[7]);
            enableSlave(slaveNumber);
            // Если текущий был отключен, но теперь включен, ничего не делать, до след. сообщения.
        } else if (strncmp(buffer, "disable ", 8) == 0) {
            uint8_t slaveNumber = atoi(&buffer[8]);
            disableSlave(slaveNumber);

            if (slaveEnabled[currentSlave] == 0) {
                uint8_t attempts = 0;
                do {
                    currentSlave = (currentSlave + 1) % NUM_SLAVES;
                    attempts++;
                    if (attempts >= NUM_SLAVES) {
                        USARTTransmitStringLnP(str_err_no_active);
                        break;
                    }
                } while (slaveEnabled[currentSlave] == 0);
            }
        } else if (strncmp(buffer, "current", 7) == 0) {
            if (slaveEnabled[currentSlave] == 1) {
                toBinary(slaveAddresses[currentSlave], binaryAddress);
                sprintf(outBuffer, "Current slave: Addr: 0x%02X (Binary: %s) [%s]",
                        slaveAddresses[currentSlave], binaryAddress, "Enabled");
                USARTTransmitStringLn(outBuffer);
            } else {
                USARTTransmitStringLnP(str_err_current_disable);
            }
        } else if (strncmp(buffer, "save", 4) == 0) {
            saveDataToEEPROM();
        } else if (strncmp(buffer, "clear", 5) == 0) {
            // Очищаем сигнатуру в EEPROM
            for (uint8_t i = 0; i < 4; i++) {
                eeprom_update_byte(&eeSignature[i], 0xFF);
            }

            USARTTransmitStringLnP(str_data_clear);
        } else if (strncmp(buffer, "reset", 5) == 0) {
            USARTTransmitStringLnP(str_reset);
            USARTTransmitStringLn("");

            softwareReset();
        } else if (strncmp(buffer, "help", 4) == 0) {
            USARTTransmitStringLnP(str_help_header);
            USARTTransmitStringLnP(str_help_print1);
            USARTTransmitStringLnP(str_help_print2);
            USARTTransmitStringLnP(str_help_enable);
            USARTTransmitStringLnP(str_help_disable);
            USARTTransmitStringLnP(str_help_current);
            USARTTransmitStringLnP(str_help_save);
            USARTTransmitStringLnP(str_help_clear);
            USARTTransmitStringLnP(str_help_reset);
            USARTTransmitStringLnP(str_help_help);
            USARTTransmitStringLn("");
            USARTTransmitStringLnP(str_help_note);
            USARTTransmitStringLnP(str_help_note2);
            USARTTransmitStringLnP(str_help_note3);
            USARTTransmitStringLnP(str_help_note4);
        } else {
            sprintf(outBuffer, "You sent: %s. It was sent to Addr: 0x%02X", buffer, slaveAddresses[currentSlave]);
            USARTTransmitStringLn(outBuffer);

            LCD_Clear();
            LCD_SetCursor(1, 1);
            sprintf(displayBuffer, "Addr: 0x%02X", slaveAddresses[currentSlave]);
            LCD_String(displayBuffer);

            LCD_SetCursor(2, 1);
            sprintf(displayBuffer, "Msg: %s", buffer);
            LCD_String(displayBuffer);

            LCD_SetCursor(3, 1);
            uint8_t transformedAddress = (slaveAddresses[currentSlave] << 1) | TW_WRITE;
            sprintf(displayBuffer, "I2C Addr: 0x%02X", transformedAddress);
            LCD_String(displayBuffer);

            TWIStart();
            TWIWrite(transformedAddress);

            LCD_SetCursor(4, 1);
            char i2cMessage[80] = "I2C Msg:";
            uint8_t i = 0;
            while (buffer[i]) {
                TWIWrite(buffer[i]);

                char temp[7];
                sprintf(temp, " 0x%02X", buffer[i]);
                if ((strlen(i2cMessage) + strlen(temp)) < sizeof(i2cMessage)) {
                    strcat(i2cMessage, temp);
                } else {
                    break;
                }
                i++;
            }

            LCD_String(i2cMessage);

            TWIStop();

            uint8_t attempts = 0;
            do {
                currentSlave = (currentSlave + 1) % NUM_SLAVES;
                attempts++;
                if (attempts >= NUM_SLAVES) {
                    USARTTransmitStringLnP(str_err_no_active);
                    break;
                }
            } while (slaveEnabled[currentSlave] == 0);

            _delay_ms(1000);
        }
    }

    return 0;
}
