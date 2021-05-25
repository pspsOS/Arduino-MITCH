//timer setup for timer0, timer1, and timer2.
//For arduino uno or any board with ATMEL 328/168.. diecimila, duemilanove, lilypad, nano, mini...

//this code will enable all three arduino timer interrupts.
//timer0 will interrupt at 10Hz
//timer1 will interrupt at 1Hz

int flag = 0;
int c = 0;
bool imubmp = true;

void setup(){
  
  Serial.begin(115200);
  
  //set pins as outputs
  pinMode(3, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);

cli();//stop interrupts

//set timer0 interrupt at 10Hz
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 195.3125;// (16.000.000 / (64 * 10)) – 1 = 24.999
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS02) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK |= (1 << OCIE0A);

sei();//allow interrupts

}//end setup

ISR(TIMER0_COMPA_vect){//timer0 interrupt 10Hz toggles pin 13
  c += 1;
  if(imubmp)
  {
    digitalWrite(3,HIGH);
    imubmp = false;
  }
  else
  {
    digitalWrite(3,LOW);
    imubmp = true;
  }
  if(c >= 40)
  {
    c = 0;
    if(flag == 1)
  {
    digitalWrite(2,LOW);
    digitalWrite(1,HIGH);
    flag = 0;
  }
  else
  {
    digitalWrite(1,LOW);
    digitalWrite(2,HIGH);
    flag = 1;
  }
  }
}

void loop(){
  //do other things here
}
