#include <IRremote.h>
#include <Servo.h>
#include <EEPROM.h>

#define SERIAL_DEBUG 0
#define BLINK_LED 1

const int PIN_IR_RECV = 7;
const int PIN_H_SERVO = 9;
const int PIN_V_SERVO = 8;
const int PIN_LED     = 13;

const unsigned long KEY_LEFT = 0xFF22DD;
const unsigned long KEY_RIGHT = 0xFFC23D;
const unsigned long KEY_UP = 0xFF629D;
const unsigned long KEY_DOWN = 0xFFA857;
const unsigned long KEY_OK = 0xFF02FD;

const int H_SERVO_POS_DEFAULT = 110;
const int V_SERVO_POS_DEFAULT = 25;

const int H_SERVO_POS_MIN = 55;
const int H_SERVO_POS_MAX = 170;
const int V_SERVO_POS_MIN = 15;
const int V_SERVO_POS_MAX = 100;

const int STEP_SIZE = 1;

IRrecv irrecv( PIN_IR_RECV );
Servo  h_servo;
Servo  v_servo;

decode_results results;
unsigned long last_result = 0;

int h_servo_pos = H_SERVO_POS_DEFAULT;
int v_servo_pos = V_SERVO_POS_DEFAULT;

int h_servo_eeprom_addr = 0;
int v_servo_eeprom_addr = 1;

unsigned long last_ir_time = 0;
unsigned long last_activity_time = 0;
unsigned long idle_time = 60000;
boolean need_to_save_to_eeprom = false;

// Function declarations
void moveToPos( int h_pos, int v_pos );
void stepLeft( int STEP_SIZE );
void stepRight( int STEP_SIZE );
void stepUp( int STEP_SIZE );
void stepDown( int STEP_SIZE );
void update_position( );

void setup()
{
#if( SERIAL_DEBUG )  
  Serial.begin( 115200 );
#endif

#if ( BLINK_LED )
  pinMode( PIN_LED, OUTPUT );
  digitalWrite( PIN_LED, LOW );
#endif

  irrecv.enableIRIn( );
  h_servo.attach( PIN_H_SERVO );
  v_servo.attach( PIN_V_SERVO );
  
  h_servo_pos = EEPROM.read( h_servo_eeprom_addr );
  v_servo_pos = EEPROM.read( v_servo_eeprom_addr );
  
  if( h_servo_pos < H_SERVO_POS_MIN || h_servo_pos > H_SERVO_POS_MAX )
  {
    h_servo_pos = ( H_SERVO_POS_MIN + H_SERVO_POS_MAX ) / 2;
    need_to_save_to_eeprom = true;
  }

  if( v_servo_pos < V_SERVO_POS_MIN || v_servo_pos > V_SERVO_POS_MAX )
  {
    v_servo_pos = ( V_SERVO_POS_MIN + V_SERVO_POS_MAX ) / 2;
    need_to_save_to_eeprom = true;
  }

  h_servo.write( h_servo_pos );
  v_servo.write( v_servo_pos );
}

void loop() 
{ 
  if( irrecv.decode( &results ) )
  {

#if( SERIAL_DEBUG )  
    Serial.print( results.decode_type );
    Serial.print( ' ' );
    Serial.print( results.address, HEX );
    Serial.print( ' ' );
    Serial.print( results.value, HEX );
    Serial.print( ' ' );
    Serial.print( results.bits );
    Serial.print( ' ' );
    Serial.print( results.rawlen );
    Serial.print( ' ' );
    Serial.println( results.overflow );
#endif

    if( results.decode_type == 3 &&
        results.bits == 32 &&
        results.rawlen == 68 )
    {
      irrecv.resume();
#if ( BLINK_LED )
      digitalWrite( PIN_LED, HIGH );
#endif      
      last_result = results.value;
      last_ir_time = millis( );
      update_position( );
#if ( BLINK_LED )
      digitalWrite( PIN_LED, LOW );
#endif      
    }
    else if ( results.decode_type == 3 &&
              results.bits == 0 &&
              results.rawlen == 4 &&
              results.value == 0xFFFFFFFF )
    {
      irrecv.resume();
#if ( BLINK_LED )
      digitalWrite( PIN_LED, HIGH );
#endif      
      last_ir_time = millis( );
      update_position( );
#if ( BLINK_LED )
      digitalWrite( PIN_LED, LOW );
#endif      
    }
  }
      
  if( last_result && ( ( millis( ) - last_ir_time ) > 1000 ) )
  {
    last_result = 0;
    last_activity_time = millis();
    need_to_save_to_eeprom = true;
  }

  if( need_to_save_to_eeprom == true )
  {
    unsigned long current_time = millis();
    if( current_time - last_activity_time > idle_time )
    {
      EEPROM.write( h_servo_eeprom_addr, h_servo_pos );
      EEPROM.write( v_servo_eeprom_addr, v_servo_pos );
      need_to_save_to_eeprom = false;
    }
  }
} 

void update_position( )
{
  switch( last_result )
  {
    case KEY_LEFT:
      stepLeft( STEP_SIZE );
      break;
  
    case KEY_RIGHT:
      stepRight( STEP_SIZE );
      break;
  
    case KEY_UP:
      stepUp( STEP_SIZE );
      break;
  
    case KEY_DOWN:
      stepDown( STEP_SIZE );
      break;

    case KEY_OK:
      moveToPos( H_SERVO_POS_DEFAULT, V_SERVO_POS_DEFAULT );
      last_result = 0;
      break;

    default:
      break;
  }

#if ( SERIAL_DEBUG )
  Serial.print(h_servo_pos);
  Serial.print("  ");
  Serial.println(v_servo_pos);
#endif

  delay( 15 );
}

void moveToPos( int h_pos, int v_pos )
{
  h_servo_pos = h_pos;
  v_servo_pos = v_pos;
  
  h_servo.write( h_servo_pos );
  v_servo.write( v_servo_pos );
}

void stepLeft( int STEP_SIZE )
{
  if( h_servo_pos > H_SERVO_POS_MIN )
  {
    h_servo_pos -= STEP_SIZE;
    if( h_servo_pos < H_SERVO_POS_MIN )
    {
      h_servo_pos = H_SERVO_POS_MIN;
    }
    h_servo.write( h_servo_pos );
  }
}

void stepRight( int STEP_SIZE )
{
  if( h_servo_pos < H_SERVO_POS_MAX )
  {
    h_servo_pos += STEP_SIZE;
    if( h_servo_pos > H_SERVO_POS_MAX )
    {
      h_servo_pos = H_SERVO_POS_MAX;
    }
    h_servo.write( h_servo_pos );
  }
}

void stepUp( int STEP_SIZE )
{
  if( v_servo_pos > V_SERVO_POS_MIN )
  {
    v_servo_pos -= STEP_SIZE;
    if( v_servo_pos < V_SERVO_POS_MIN )
    {
      v_servo_pos = V_SERVO_POS_MIN;
    }
    v_servo.write( v_servo_pos );
  }
}

void stepDown( int STEP_SIZE )
{
  if( v_servo_pos < V_SERVO_POS_MAX )
  {
    v_servo_pos += STEP_SIZE;
    if( v_servo_pos > V_SERVO_POS_MAX )
    {
      v_servo_pos = V_SERVO_POS_MAX;
    }
    v_servo.write( v_servo_pos );
  }
}

