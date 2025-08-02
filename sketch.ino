// =============================================
// PIN DEFINITIONS
// =============================================

#define LDR1 13 // LDR Light sensor from traffic light 1 connected in pin A0
#define LDR2 12 // LDR Light sensor from traffic light 2 connected in pin A1
#define CO2 14  // CO2 sensor connected in pin A3
#define P1 1    // Traffic light 1 button connected in pin 1
#define P2 2    // Traffic light 2 button connected in pin 2
#define CNY1 42 // Infrared sensor 1 in traffic light 1 connected in pin 42
#define CNY2 41 // Infrared sensor 2 in traffic light 1 connected in pin 41
#define CNY3 40 // Infrared sensor 3 in traffic light 1 connected in pin 40
#define CNY4 39 // Infrared sensor 4 in traffic light 2 connected in pin 39
#define CNY5 38 // Infrared sensor 5 in traffic light 2 connected in pin 38
#define CNY6 37 // Infrared sensor 6 in traffic light 2 connected in pin 37
#define LR1 5   // Red traffic light 1 connected in pin 5
#define LY1 4   // Yellow traffic light 1 connected in pin 4
#define LG1 6   // Green traffic light 1 connected in pin 6
#define LR2 7   // Red traffic light 2 connected in pin 7
#define LY2 15  // Yellow traffic light 2 connected in pin 15
#define LG2 16  // Green traffic light 2 connected in pin 16

// =============================================
// TYPES (ENUM)
// =============================================

// 
enum TrafficLightState {
    RED,
    YELLOW,
    GREEN,
    OFF
  };

enum IntersectionState {
    G1R2,  // TL1 Verde, TL2 Rojo
    Y1R2,  // TL1 Amarillo, TL2 Rojo  
    R1R2,  // TL1 Rojo, TL2 Rojo (transición)
    R1G2,  // TL1 Rojo, TL2 Verde
    R1Y2   // TL1 Rojo, TL2 Amarillo
  };

// =============================================
// CONFIGURATION STRUCTURES
// =============================================

//Traffic light structs (TL)
struct TrafficLightTimes {
    int minGreenTime;
    int greenTime1;
    int greenTime2;
    int yellow2RedTime;
    int red2GreenTime;
    int rainyExtraTime;
    int blinkTime;
    int walkerTime;
};

enum IntersectionMode {
    NORMAL,
    NIGHT,
    EMERGENCY
};

// =============================================
// CLASSES
// =============================================

// Base Sensor Class
class Sensor {
    protected:
      int pin;
      String name;
      
    public:
      Sensor(int sensorPin, String sensorName) : pin(sensorPin), name(sensorName) {
        pinMode(pin, INPUT);
      }
      
      virtual int readValue() = 0;
      virtual String getName() { return name; }
    };
    
    // Light Sensor Class
class LightSensor : public Sensor {
    public:
      LightSensor(int pin, String name) : Sensor(pin, name) {}
      
      int readValue() override {
        return analogRead(pin);
      }
      
      bool isDark() {
        return readValue() < 300;
      }
    };
    
    // Infrared Traffic Sensor Class
class TrafficSensor : public Sensor {
    public:
      TrafficSensor(int pin, String name) : Sensor(pin, name) {}
      
      int readValue() override {
        return digitalRead(pin);
      }
      
      bool vehicleDetected() {
        return readValue() == 0;
      }
    };
    
    // CO2 Sensor Class
class CO2Sensor : public Sensor {
    public:
      CO2Sensor(int pin, String name) : Sensor(pin, name) {}
      
      int readValue() override {
        return analogRead(pin);
      }
      
      bool highPollution() {
        return readValue() > 600;
      }
};

class WalkerButton : public Sensor {
  private:
    int lastValue;
    bool wasPressed;
    bool isPressed;
    int numPresses;
    int readValue() override {
      return digitalRead(pin);
    }
  public:
    WalkerButton(int pin, String name) : Sensor(pin, name), lastValue(0) {}
  
    void update() {
      if (readValue() != lastValue) {
        lastValue = readValue();
        wasPressed = true;
        numPresses++;
      }
    }

    void reset() {
      wasPressed = false;
      numPresses = 0;
    }

    bool getIsPressed() {
      return isPressed;
    }

    bool getWasPressed() {
      return wasPressed;
    }

    int getNumPresses() {
      return numPresses;
    }
};

class TrafficLight {
    private:
      int redPin;
      int yellowPin;
      int greenPin;
      TrafficLightState state;
      
    public:
      TrafficLight(int red, int yellow, int green)
        : redPin(red), yellowPin(yellow), greenPin(green), state(RED) {
        pinMode(redPin, OUTPUT);
        pinMode(yellowPin, OUTPUT);
        pinMode(greenPin, OUTPUT);
        update();
      }
      
      void update() {
        digitalWrite(redPin, LOW);
        digitalWrite(yellowPin, LOW);
        digitalWrite(greenPin, LOW);
        
        switch (state) {
          case GREEN:
            digitalWrite(greenPin, HIGH);
            break;
          case YELLOW:
            digitalWrite(yellowPin, HIGH);
            break;
          case RED:
            digitalWrite(redPin, HIGH);
            break;
        case OFF:
            // All lights off
            break;
        }
      }
      
      
      void setState(TrafficLightState newState) {
        state = newState;
        update();
      }
      
      TrafficLightState getState() {
        return state;
      }

    };

class Intersection {
  private:
    TrafficLight* TL1;
    TrafficLight* TL2;
    TrafficSensor* TS1;
    TrafficSensor* TS2;
    TrafficSensor* TS3;
    TrafficSensor* TS4;
    TrafficSensor* TS5;
    TrafficSensor* TS6;
    IntersectionState state;
    IntersectionState lastState;
    IntersectionMode mode;
    unsigned long lastStateChange;
    TrafficLightTimes timings;
    bool blink;
    bool isRainy;
    WalkerButton* walkerButton1;
    WalkerButton* walkerButton2;
    
  public:
    Intersection(TrafficLight* tl1, TrafficLight* tl2, 
        TrafficSensor* ts1, TrafficSensor* ts2, TrafficSensor* ts3, TrafficSensor* ts4, 
        TrafficSensor* ts5, TrafficSensor* ts6, WalkerButton* wb1, WalkerButton* wb2, TrafficLightTimes tlt) 
    : TL1(tl1), TL2(tl2), TS1(ts1), TS2(ts2), TS3(ts3), TS4(ts4), TS5(ts5), TS6(ts6), 
      state(G1R2), mode(NORMAL), walkerButton1(wb1), walkerButton2(wb2), timings(tlt), blink(false), isRainy(false) {

      lastStateChange = millis();
      lastState = state;
      updateTrafficLights();
    }
    
    void updateTrafficLights() {
      switch (state) {
        case G1R2:  // TL1 Verde, TL2 Rojo
          TL1->setState(GREEN);
          TL2->setState(RED);
          break;
          
        case Y1R2:  // TL1 Amarillo, TL2 Rojo
          TL1->setState(YELLOW);
          TL2->setState(RED);
          break;
          
        case R1R2:
          TL1->setState(RED);
          TL2->setState(RED);
          break;
          
        case R1G2:  // TL1 Rojo, TL2 Verde
          TL1->setState(RED);
          TL2->setState(GREEN);
          break;
          
        case R1Y2:  // TL1 Rojo, TL2 Amarillo
          TL1->setState(RED);
          TL2->setState(YELLOW);
          break;
      }
    }
    
    void update() {
      unsigned long currentTime = millis();
      unsigned long timeSinceLastChange = currentTime - lastStateChange;
      unsigned int trafficCount1 = 0;
      unsigned int trafficCount2 = 0;

    // Check if any traffic sensors detect vehicles
      if (TS1->vehicleDetected()) trafficCount1++;
      if (TS2->vehicleDetected()) trafficCount1++;
      if (TS3->vehicleDetected()) trafficCount1++;
      if (TS4->vehicleDetected()) trafficCount2++;
      if (TS5->vehicleDetected()) trafficCount2++;
      if (TS6->vehicleDetected()) trafficCount2++;


      switch (mode) {
        case NORMAL:
          switch (state) {
            case G1R2: // tiempo verde TL1
              if (timeSinceLastChange >= timings.greenTime1 + (walkerButton2->getWasPressed() ? timings.walkerTime : 0)) {
                walkerButton2->reset();
                state = Y1R2;
                lastState = G1R2;
                lastStateChange = millis();
              }
              break;
              
            case Y1R2: // tiempo yellow2red and rainyyellow
              if (timeSinceLastChange >= (timings.yellow2RedTime + (isRainy ? timings.rainyExtraTime : 0))) {
                state = R1R2;
                lastState = Y1R2;
                lastStateChange = millis();
              }
              break;

            case R1R2: // tiempo yellow2red and rainyyellow
                if (lastState == Y1R2) {
                    if (timeSinceLastChange >= (timings.red2GreenTime + (isRainy ? timings.rainyExtraTime : 0))) {
                        state = R1G2;
                        lastStateChange = millis();
                    }
                }
                if (lastState == R1Y2) {
                    if (timeSinceLastChange >= (timings.red2GreenTime + (isRainy ? timings.rainyExtraTime : 0))) {
                        state = G1R2;
                        lastStateChange = millis();
                    }
                }
                break;

            case R1G2: // tiempo verde TL2
              if (timeSinceLastChange >= timings.greenTime2 + (walkerButton1->getWasPressed() ? timings.walkerTime : 0)) {
                walkerButton1->reset();
                state = R1Y2;
                lastState = R1G2;
                lastStateChange = millis();
              }
              break;
              
            case R1Y2: // tiempo yellow2red and rainyyellow
              if (timeSinceLastChange >= (timings.yellow2RedTime + (isRainy ? timings.rainyExtraTime : 0))) {
                state = R1R2;
                lastState = R1Y2;
                lastStateChange = millis();
              }
              break;
          }
          updateTrafficLights();
          break;
          
        case NIGHT:
            if (trafficCount1 > 0 || trafficCount2 > 0) {
                // Si hay tráfico, cambiar a modo normal
                if (trafficCount1 > trafficCount2) {
                    state = G1R2; // Priorizar TL1
                } else {
                    state = R1G2; // Priorizar TL2
                }
                lastStateChange = millis();
                updateTrafficLights();
            } else {
                if(timeSinceLastChange >= timings.blinkTime){
                    blink = !blink;
                    if (blink){
                        TL1->setState(YELLOW);
                        TL2->setState(YELLOW);
                    }else{
                        TL1->setState(OFF);
                        TL2->setState(OFF);
                    }
                    lastStateChange = millis();
                }
            }
          break;
          
        case EMERGENCY:
            switch(state){
                case G1R2: // cambio a amarillo TL1
                    state = Y1R2;
                    lastStateChange = millis();
                break;
                
                case Y1R2: // tiempo yellow2red or rainyyellow
                if (timeSinceLastChange >= timings.yellow2RedTime) {
                    state = R1G2;
                    lastStateChange = millis();
                }
                break;
                
                case R1G2:
                    break;
                
                case R1Y2: // vuelve a R1G2
                    state = R1G2;
                    lastStateChange = millis();
                break;   
            }
            updateTrafficLights();
          break;
      }
      
    }
    
    void setMode(IntersectionMode newMode) {
      if (mode == newMode) return; // No cambiar si es el mismo modo
      
      IntersectionMode previousMode = mode;
      mode = newMode;
      lastStateChange = millis();
      
      // Manejar transiciones específicas entre modos
      switch (previousMode) {
        case NORMAL:
          switch (newMode) {
            case NIGHT:
              // Transición de NORMAL a NIGHT: inicializar parpadeo
              blink = false;
              break;
            case EMERGENCY:
              // Transición de NORMAL a EMERGENCY: activar emergencia inmediatamente
              break;
          }
          break;
          
        case NIGHT:
          switch (newMode) {
            case NORMAL:
              // Transición de NIGHT a NORMAL: reiniciar ciclo normal
              state = G1R2;
              lastStateChange = millis();
              updateTrafficLights();
              break;
            case EMERGENCY:
              state = R1G2;
              lastStateChange = millis();
              updateTrafficLights();
              break;
          }
          break;
          
        case EMERGENCY:
          switch (newMode) {
            case NORMAL:
              // Transición de EMERGENCY a NORMAL: reiniciar ciclo
              state = R1Y2;
              lastStateChange = millis();
              updateTrafficLights();
              break;
            case NIGHT:
              // Transición de EMERGENCY a NIGHT: inicializar parpadeo
              blink = false;
              break;
          }
          break;
      }
    }
    
    
    IntersectionState getCurrentState() {
      return state;
    }
    
    IntersectionMode getCurrentMode() {
      return mode;
    }

};

class City {
  private:
    Intersection* intersection;
    LightSensor* lightSensor1;
    LightSensor* lightSensor2;
    CO2Sensor* co2Sensor;
    TrafficSensor* trafficSensors[6];
    IntersectionMode mode;
    
  public:
    City(Intersection* intersec, LightSensor* ls1, LightSensor* ls2, CO2Sensor* co2,
         TrafficSensor* ts1, TrafficSensor* ts2, TrafficSensor* ts3,
         TrafficSensor* ts4, TrafficSensor* ts5, TrafficSensor* ts6) 
      : intersection(intersec), lightSensor1(ls1), lightSensor2(ls2), co2Sensor(co2), mode(NORMAL) {
      trafficSensors[0] = ts1;
      trafficSensors[1] = ts2;
      trafficSensors[2] = ts3;
      trafficSensors[3] = ts4;
      trafficSensors[4] = ts5;
      trafficSensors[5] = ts6;
    }

    void update(){
        if(isDark() && mode !=  NIGHT){
            mode= NIGHT;
            intersection->setMode(  NIGHT);
            Serial.println("  NIGHT MODE ACTIVATED");
        }else if(!isDark() && mode ==   NIGHT){
            mode=NORMAL;
            intersection->setMode(NORMAL);
            Serial.println("NORMAL MODE ACTIVATED");
        }
        if (highPollution() && mode != EMERGENCY){
            mode = EMERGENCY;
            intersection->setMode(EMERGENCY);
            Serial.println("EMERGENCY MODE ACTIVATED");
        }else if(!highPollution() && mode == EMERGENCY){
            mode = NORMAL;
            intersection->setMode(NORMAL);
            Serial.println("NORMAL MODE ACTIVATED");
        }
        intersection->update();
    }

    bool isDark(){
        return (lightSensor1->isDark() || lightSensor2->isDark());
    }

    bool highPollution(){
        return(co2Sensor->highPollution());
    }
};

// =============================================
// GLOBAL VARIABLES
// =============================================

// Sensor objects
LightSensor lightSensor1(LDR1, "Light Sensor 1");
LightSensor lightSensor2(LDR2, "Light Sensor 2"); 
CO2Sensor co2Sensor(CO2, "CO2 Sensor");
TrafficSensor trafficSensor1(CNY1, "Traffic Sensor 1");
TrafficSensor trafficSensor2(CNY2, "Traffic Sensor 2");
TrafficSensor trafficSensor3(CNY3, "Traffic Sensor 3");
TrafficSensor trafficSensor4(CNY4, "Traffic Sensor 4");
TrafficSensor trafficSensor5(CNY5, "Traffic Sensor 5");
TrafficSensor trafficSensor6(CNY6, "Traffic Sensor 6");


// Create traffic light objects
TrafficLight tl1(LR1, LY1, LG1);  // Traffic Light 1: pins 5, 4, 6
TrafficLight tl2(LR2, LY2, LG2);  // Traffic Light 2: pins 7, 15, 16

// Configure timing
TrafficLightTimes timings = {
  3000,  // minGreenTime (3 seconds)
  5000,  // greenTime1 (5 seconds)
  4000,  // greenTime2 (4 seconds) 
  2000,  // yellow2RedTime (2 seconds)
  2000,  // red2GreenTime (1 second)
  1000,  // rainyExtraTime (1 second)
  500,   // blinkTime (0.5 seconds)
  30000   // walkerTime (30 seconds)
};

// Walker buttons
WalkerButton walkerButton1(P1, "Walker Button 1");
WalkerButton walkerButton2(P2, "Walker Button 2");

// Create intersection
Intersection intersection(&tl1, &tl2, &trafficSensor1, &trafficSensor2, 
    &trafficSensor3, &trafficSensor4, &trafficSensor5, &trafficSensor6, 
     &walkerButton1, &walkerButton2, timings);

// Create city object
City city(&intersection, &lightSensor1, &lightSensor2, &co2Sensor,
          &trafficSensor1, &trafficSensor2, &trafficSensor3,
          &trafficSensor4, &trafficSensor5, &trafficSensor6);

// =============================================
// ARDUINO SETUP AND LOOP
// =============================================

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Traffic Light System Initialized");
  Serial.println("Mode: NORMAL");
  
  // Traffic lights are automatically initialized in their constructors
  // Intersection starts in NORMAL mode by default
}

void loop() {
  // Update intersection (handles all traffic light logic)
  city.update();
  walkerButton1.update();
  walkerButton2.update();
  // Small delay to prevent overwhelming the system
  delay(10);
}