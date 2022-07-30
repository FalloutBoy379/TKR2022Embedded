void setup()
{
  // setup serial object
  Serial.begin(9600);
}

void loop()
{
  //Have the arduino wait till it recieves input
  while (Serial.available() == 0);

  //read input
  // -'0' for easier conversion between char and decimal, arduino takes int val to be char and to counter it and recieve same echo we subtract another char 0(denoted by '0') from it
  int val = Serial.read() - '0';

  //Echo the input
  Serial.println(val);
  Serial.flush();
}
