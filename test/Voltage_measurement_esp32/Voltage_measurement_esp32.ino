//THERE IS NO WARRANTY FOR THE SOFTWARE, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR 
//OTHER PARTIES PROVIDE THE SOFTWARE “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
//OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE SOFTWARE IS WITH THE CUSTOMER. SHOULD THE 
//SOFTWARE PROVE DEFECTIVE, THE CUSTOMER ASSUMES THE COST OF ALL NECESSARY SERVICING, REPAIR, OR CORRECTION EXCEPT TO THE EXTENT SET OUT UNDER THE HARDWARE WARRANTY IN THESE TERMS.

void setup() {
   Serial.begin(115200);
}

void loop() {
  //                     Add a voltage divider to the ADC input channel R1 is typically 100K and R2 calculated to achieved 3.3v output as the input to the ADC
  //                     analogRead(ADC pin) / ADC resolution * Voltage Range * Required Value of R2 / Preferred Value of R2
  //                     In this example the input voltage for measurement is 15v
  //                     15v----100K+--to ADC Input GPIO36
  //                                |
  //                               (18K+18K=36K)
  //                                |
  //                               Gnd
  //float voltage_raw = analogRead(36);
  float voltage= (analogRead(36)/4096)*12.46*(35.9/36);
   Serial.print(voltage);
  Serial.println("v");
  delay(200);
}
