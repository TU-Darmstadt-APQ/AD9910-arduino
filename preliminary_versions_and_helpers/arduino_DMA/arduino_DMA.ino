// Enable the AHB DMAC for memory to memory transfer
uint32_t data[] = { 0x000ff1fe, 0x0, 0x000ff1fe, 0x0,0x000ff1fe, 0x0, 0x000ff1fe, 0x0,0x000ff1fe, 0x0, 0x000ff1fe, 0x0,0x000ff1fe, 0x0, 0x000ff1fe, 0x0};
uint32_t destData[16];

void setup() {
  Serial.begin(115200);                                                        // Initialise the native USB port
  while(!Serial);                                                              // Wait for the console to be ready

  int PP_Pins[] = {33,34,35,36,37,38,39,40,44,45,46,47,48,49,50,51};
        
  for(int a = 0; a < (sizeof(PP_Pins)/sizeof(PP_Pins[0])); a++) {
      pinMode(PP_Pins[a], OUTPUT);
      digitalWrite(PP_Pins[a], LOW); 
  }
//  pinMode(2, OUTPUT);
//  pinMode(3, OUTPUT);
//  pinMode(53, OUTPUT);
//  // Configure Parallel Port Destination to frequency:
//  digitalWrite(2, LOW);
//  digitalWrite(3, HIGH);
  // Mask the output for the corresponding pins; BE CAREFUL: Interrupts can destroy this!
  REG_PIOC_OWER = 0x000ff1fe; // Enable output writing for all PP_Pins as HEX: 0x000FF1FE
  REG_PIOC_OWDR = 0xfff00e01; // Disable output writing for all other Pins

  delay(1000);
  PIOC->PIO_SODR = 0x000ff1fe;
  PIOC->PIO_CODR = 0x000ff1fe;

  
  PMC->PMC_PCER1 |= PMC_PCER1_PID39;                                              // Enable the DMAC
  DMAC->DMAC_EN = DMAC_EN_ENABLE;                          
  DMAC->DMAC_CHDR = DMAC_CHDR_DIS3;                                               // Disble DMAC channel 3
  DMAC->DMAC_EBCISR;                                                              // Clear any pending interrupts
  DMAC->DMAC_CH_NUM[3].DMAC_SADDR = (uint32_t)data;                               // Set source address to data array in memory
  DMAC->DMAC_CH_NUM[3].DMAC_DADDR = (uint32_t)destData;                         // Set destination address to data array in memory
  //DMAC->DMAC_CH_NUM[3].DMAC_DADDR = (PIOC->PIO_ODSR);                             // Set destination address to PIO register
  DMAC->DMAC_CH_NUM[3].DMAC_DSCR = 0;                                             // Set the descriptor to 0
  DMAC->DMAC_CH_NUM[3].DMAC_CTRLA = DMAC_CTRLA_BTSIZE(16) |                        // Set the beat size to 16
                                    DMAC_CTRLA_SRC_WIDTH_WORD |                   // Set the source data size to word (32-bits)
                                    DMAC_CTRLA_DST_WIDTH_WORD;                    // Set the destination data size to word (32-bits)
  DMAC->DMAC_CH_NUM[3].DMAC_CTRLB = DMAC_CTRLB_SRC_DSCR_FETCH_DISABLE |           // Use DMAC registers for the source descriptor
                                    DMAC_CTRLB_DST_DSCR_FETCH_DISABLE |           // Use DMAC registers for the destination descriptor
                                    DMAC_CTRLB_FC_MEM2MEM_DMA_FC |                // Set-up the transfer from memory to memory
                                    DMAC_CTRLB_SRC_INCR_INCREMENTING |            // Increment the source address for each beat
                                    DMAC_CTRLB_DST_INCR_INCREMENTING;                    // Keep the destination address fixed for each beat
  DMAC->DMAC_CH_NUM[3].DMAC_CFG = //DMAC_CFG_DST_PER(15) |                        // Set the destination trigger to the PIOC
                                  //DMAC_CFG_DST_H2SEL |                            // Activate the destination hardware handshaking interface
                                  DMAC_CFG_SOD |                                  // Enable Stop On Done (SOD)
                                  //DMAC_CFG_AHB_PROT(1) |                          // Set the AHB bus protection
                                  //DMAC_CFG_FIFOCFG_ASAP_CFG;                      // Send AHB bus transfer to destination ASAP
                                  DMAC_CFG_FIFOCFG_ALAP_CFG;                      // Send largest AHB bus transfer to destination
  DMAC->DMAC_CHER = DMAC_CHER_ENA3;                                               // Enable the AHB DMA Controller
  while (DMAC->DMAC_CHSR & DMAC_CHSR_ENA3);                                       // Wait for the AHB DMA Controller to complete
  PIOC->PIO_SODR = 0x000ff1fe;
  PIOC->PIO_CODR = 0x000ff1fe;
  
  for (uint8_t i = 0; i < 16; i++)
  {
    Serial.println(destData[i]);                                               // Display the results in the destintation array
  }
}

void loop() {
  //Serial.println(PIOC->PIO_ODSR);
  //Serial.println('bla');
  }
