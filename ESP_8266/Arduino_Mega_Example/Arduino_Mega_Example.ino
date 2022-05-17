void setup() {
  Serial.begin(115200);
  
  Serial1.begin(115200);
  Serial1.setTimeout(10);
}
  

void loop() {
  String s = Serial1.readString();
  if(s.length() != 0)
  {
    float d = (s.substring(s.indexOf('d')+4,s.indexOf(';'))).toFloat();
    float theta = (s.substring(s.indexOf('a')+4,s.indexOf('/'))).toFloat();
    Serial.println(d);
    Serial.println(theta);
  }
}
