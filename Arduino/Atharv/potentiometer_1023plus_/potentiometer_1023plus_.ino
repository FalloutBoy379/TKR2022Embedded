int infPot = 0
int previousData=0;
int currentData=0;
int totalData=0;
int n=0;
int m=0;

void setup()
{
  Serial.begin(9600)
  pinMode(infPot,INPUT)
  previousData= analogRead(infPot);
}

void loop()
{
  currentData = analogRead(infPot);
  if(previousData != currentData)
  {
   if(previousData < currentData)
   {
     
   }
  }
}
