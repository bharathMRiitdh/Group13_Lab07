#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define SW1    (1U << 0)  // PF0 (SW1)
#define SW2    (1U << 4)  // PF4 (SW2)

// Define LED pins
#define RED_LED    (1U << 1) // PF1
#define BLUE_LED   (1U << 2) // PF2
#define GREEN_LED  (1U << 3) // PF3

uint8_t receivedByte;
bool dataReceivedFlag = true;

void Uart3_send(void);
void UART3_Transmit(uint8_t data);

void PortF_Initialisation(void){

    // PORTF, PF7-PF0, PF4-SW1, PF3-green, PF2-blue, PF1-red, PF0-SW2
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;   // Enable clock for Port F
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;      // Unlock Port F
    GPIO_PORTF_CR_R = 0x1f;                 // Commit changes, 1-enable (PF7-PF0 = 00011111)
    GPIO_PORTF_DEN_R = 0x1f;                // Digital function enable, 1-enable (PF7-PF0 = 00011111)
    GPIO_PORTF_DIR_R = 0x0e;                // Set output/input, 1-output (PF7-PF0 = 00001110)
    GPIO_PORTF_PUR_R = 0x11;                // Enable pull-up resistor, 1-enable (PF7-PF0 = 00010001)
    GPIO_PORTF_DATA_R = 0x00;               // Reset the data register (PF7-PF0 = 00000000)

}

void PortC_Initialisation(void) {

    // Enable the clock for UART3 and GPIO Port C
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOC;     // Enable GPIO Port C clock
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R3;  // Enable clock for UART3
    while ((SYSCTL_PRUART_R & SYSCTL_PRUART_R3) == 0) {};  // Wait for UART3 to be ready


    // Configure PC6 (RX) and PC7 (TX)
    GPIO_PORTC_LOCK_R = GPIO_LOCK_KEY;      // Unlock Port C
    GPIO_PORTC_CR_R = 0xC0;                 // Commit changes, enable (PC7-PC6 = 11000000)
    GPIO_PORTC_DEN_R = 0xC0;                // Digital enable PC6 and PC7
    GPIO_PORTC_AFSEL_R = 0xC0;              // Enable alternate function for PC6 and PC7
    GPIO_PORTC_AMSEL_R = 0x00;              // Turn off analog function
    GPIO_PORTC_PCTL_R &= ~((0xF << 24) | (0xF << 28)); // Clear PCTL for PC6 and PC7
    GPIO_PORTC_PCTL_R |= ((0x1 << 24) | (0x1 << 28));  // Set PCTL to UART3 for PC6 Rx and PC7 Tx
}

void Uart3_Initialisation(void) {

    // Configure UART3 for 9600 baud rate, 8 data bits, odd parity
    UART3_CTL_R = 0x00;                     // Disable UART before configuration
    UART3_IBRD_R = 104;                     // Integer part of BRD = 16MHz / (16 * 9600) = 104
    UART3_FBRD_R = 11;                      // Fractional part of BRD = 0.16 * 64 + 0.5 = 11
    UART3_CC_R = 0x00;                      // Use system clock
    UART3_LCRH_R = 0x72;                    // 8 bits, odd parity
    UART3_CTL_R = 0x301;                    // Enable UART
}

uint8_t UART3_ReceiveByte(void) {

    while ((UART3_FR_R & 0x10) != 0) // Wait until RXFE is 0
    {
        Uart3_send();
    }
    return UART3_DR_R; // Read data
}

void Uart3_Read(void){

    receivedByte =  UART3_ReceiveByte();

    if(dataReceivedFlag){
        // Check if a parity error occurred
        if (UART3_FR_R & 0x04) { // Check PE (Parity Error) bit
                    GPIO_PORTF_DATA_R &= ~GREEN_LED;  // Turn off green LED
                    GPIO_PORTF_DATA_R &= ~BLUE_LED;   // Turn off blue LED
                    GPIO_PORTF_DATA_R |= RED_LED;     // Turn on red LED (error)
        } else {
            switch (receivedByte) {
                case 0xAA:
                    GPIO_PORTF_DATA_R |= GREEN_LED;  // Turn on green LED
                    GPIO_PORTF_DATA_R &= ~BLUE_LED;   // Turn off blue LED
                    GPIO_PORTF_DATA_R &= ~RED_LED;     // Turn off red LED (error)
                    break;
                case 0xF0:
                    GPIO_PORTF_DATA_R &= ~GREEN_LED;  // Turn off green LED
                    GPIO_PORTF_DATA_R |= BLUE_LED;   // Turn on blue LED
                    GPIO_PORTF_DATA_R &= ~RED_LED;     // Turn off red LED (error)
                    break;
                default:
                    GPIO_PORTF_DATA_R &= ~GREEN_LED;  // Turn off green LED
                    GPIO_PORTF_DATA_R &= ~BLUE_LED;   // Turn off blue LED
                    GPIO_PORTF_DATA_R |= RED_LED;     // Turn on red LED (error)
                    break;
            }
        }
    }
}

void Uart3_send(void){

    if (!(GPIO_PORTF_DATA_R & SW1)) { // SW1 pressed
        UART3_Transmit(0xF0);
        while (!(GPIO_PORTF_DATA_R & SW1)); // Wait until released
    }
    if (!(GPIO_PORTF_DATA_R & SW2)) { // SW2 pressed
        UART3_Transmit(0xAA);
        while (!(GPIO_PORTF_DATA_R & SW2)); // Wait until released
    }
}

void UART3_Transmit(uint8_t data) {
    while (UART3_FR_R & UART_FR_TXFF);  // Wait until the transmit FIFO is not full
    UART3_DR_R = data;                  // Transmit data
}

int main(void) {
    PortF_Initialisation();
    PortC_Initialisation();
    Uart3_Initialisation();

    while (1) {
        Uart3_Read();
    }
}
