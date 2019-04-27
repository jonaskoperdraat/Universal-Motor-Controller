/**********************************************************************************************
 * Arduino Universal Motor Controller - Version 1.0.2
 * by teo Basili <basili.teo@gmail.com> https://github.com/teo666
 **********************************************************************************************/

//#define TEST_MODE

#include "PID_ASYNC.h"
#include "pin_definition.h"
#include <EEPROM.h>

PID *motor_PID;

volatile uint16_t zcd_tick_log = 0;
volatile uint16_t tacho_tick_log = 0;
volatile uint16_t tick_after_zcd = 0;
volatile uint16_t tick_after_tacho = 0;
volatile uint8_t triac_state = 0;

// questa cosa del delay count mi serve per evitare di usare le funzioni di
// tempo di arduino
volatile uint8_t delay_count = 0;
volatile uint8_t delay_allow = 0;

uint16_t tick_per_phase = 0;

/*in alcune circostanze (dovute a problemi induttivi dal momento che ho potuto
registrarli solamente a motore connesso???) capita che il circuito zcd presenti
dei disturbi che fanno si' che alla sua uscita compaiano dei picchi di tensione
che alterano il corretto fasamento; questi picchi tuttavia hanno una durata
molto breve rispetto al tempo in cui l'onda quadra dello zcd si trova a livello
logico alto, per cui utilizzo questa variabile per distinguere tali valori

- al cambiamento dello stato di uscita dello zdc azzero dei parametri, compreso
zcd_error_correction
- controllo periodicamente (nell-interrupt del timer) il valore di ingresso
dello zcd: se e' HIGH incremento zcd_error_correction altrimenti non faccio
niente
- se dopo un po' di tempo tale valore raggiunge un valore limite (10) allora la
mi sto trovando nel punto un cui lo zcd e' effettivamente in fase e non si e'
presentato il disturbo
-- Google Tranlate IT > EN --
in some circumstances (due to inductive problems since I could
register them only when the motor is connected ???) it happens that the zcd circuit is present
of the disturbances which cause voltage peaks to appear at its output
that alter the correct phasing; these peaks however have a duration
very short compared to the time when the zcd square wave is level
high logic, so I use this variable to distinguish these values
- when the output state of the zdc changes, the parameters are reset, including
zcd_error_correction
- check the input value periodically (in the timer interrupt)
del zcd: if it is HIGH increment zcd_error_correction otherwise I don't
anything
- if after a while this value reaches a limit value (10) then la
I'm finding myself at the point where the zcd is actually in phase and it's not
presented the disorder
*/
volatile uint8_t zcd_error_correction = 0;
volatile uint8_t found_correct_main_phase = 0;

volatile uint8_t tacho_error_correction = 0;
volatile uint8_t found_correct_tacho_phase = 0;

/*
variabili per la lettura dei valori analogici, contengono il valore delle
letture
-- Google Translate IT > EN --
variables for reading analog values, contain the value of
readings
*/
volatile uint8_t HIGH_ANALOG_REG = 0;
volatile uint8_t LOW_ANALOG_REG = 0;

volatile uint8_t frequency_calc_added = 0;

// il range dell' output va da 0 a 485, a 0 il motore gira a foo 485 val minimo
// -- Google Translate IT > EN --
// the range of the output goes from 0 to 485, to 0 the motor turns at foo 485 minimum value
volatile uint16_t output = 65535; // max value
volatile uint16_t tacho_min_speed_value = 0;
volatile uint16_t tacho_max_speed_value = 0;

uint16_t output_min_speed_value = 0;
uint16_t output_max_speed_value = 0;

// variabili del pid
// -- translate IT > EN
// PID variables
double Setpoint, Input, Output;
volatile uint8_t computeBarrier = 0;

volatile uint16_t delay_counter = 0;
volatile uint16_t last_delay_counter = 0;

//////////////////// UTILITY FUNCTIONS ///////////////////////

uint8_t my_digital_read(uint8_t port_reg, uint8_t bit) {
  if (port_reg & _BV(bit))
    return 1;
  return 0;
}

void limit(volatile uint16_t *val, uint16_t min, uint16_t max) {
  if (*val > max) {
    *val = max;
  } else if (*val < min) {
    *val = min;
  }
}

uint16_t calculate_main_power_frequency() {
  uint8_t frequency_calc = 0;
  uint32_t tick_sum = 0;

  while (frequency_calc < 20) {
    if (frequency_calc_added) {
      frequency_calc_added = 0;
      tick_sum += zcd_tick_log;
      frequency_calc++;
    }
  }

  return tick_sum / 20;
}

void check_programming_button() {
  // se il bottone e' pigiato chiededre di rilasciarlo
  if (!my_digital_read(PIND, PROG_PIN)) {
    Serial.println("Release programming button, please");
    while (!my_digital_read(PIND, PROG_PIN)) {
    }
  }
}

void button_hold_request(void (*loop)(), void (*on_exit)()) {
  while (1) {
    loop();
    if (delay_allow) {
      on_exit();
      break;
    }
  }
}

void loop_read_fun() {
  output =
      map((HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG, 1023, 0, 0, tick_per_phase);
}

//////////////////////////////////////////////////////////////////////

void save_low_speed_exit_fun() {
  tacho_min_speed_value = tacho_tick_log;
  output_min_speed_value = output;
  EEPROM.write(0, tacho_min_speed_value >> 8);
  EEPROM.write(1, tacho_min_speed_value);
  EEPROM.write(2, output_min_speed_value >> 8);
  EEPROM.write(3, output_min_speed_value);
  Serial.print(F("LOW speed tacho set to: "));
  Serial.print(tacho_min_speed_value);
  Serial.print(F(" LOW speed output set to: "));
  Serial.print(output_min_speed_value);
  Serial.println(F(", saving ..."));
}

void save_high_speed_exit_fun() {
  tacho_max_speed_value = tacho_tick_log;
  output_max_speed_value = output;
  if (tacho_max_speed_value > tacho_min_speed_value) {
    Serial.println(F("The HIGH speed must be higher than LOW speed"));
    // se il bottone e' pigiato chiededre di rilasciarlo
    check_programming_button();
    Serial.println(F("try again"));
  } else {
    EEPROM.write(4, tacho_max_speed_value >> 8);
    EEPROM.write(5, tacho_max_speed_value);
    EEPROM.write(6, output_max_speed_value >> 8);
    EEPROM.write(7, output_max_speed_value);
    Serial.print(F("HIGH speed tacho set to: "));
    Serial.print(tacho_max_speed_value);
    Serial.print(F(" HIGH speed output set to: "));
    Serial.print(output_max_speed_value);
    Serial.println(F(", saving ..."));
  }
}

#ifdef TEST_MODE

Coefficient k_param;

String serial_read;

// funzione di ricerca dei parametri del pid
// -- Google Translate IT > EN
// search function for pid parameters
CoefficientPtr search() { return &k_param; }

#else
#include "configuration.h"
#endif

void setup() {

  INIT_TRIAC_LOG();
  INIT_ZCD_LOG();
  INIT_TACHO_LOG();

  INIT_TRIAC();
  INIT_PROG_BUTTON();
  INIT_ZCD();
  INIT_TACHO();

  // abilita le interruzioni sul pin 3 e 2 di arduino (bit meno significativo e'
  // il 2) pin 3 = int1 pin 2 = int0
  // -- Google Translate IT > EN
  // enable the interruptions on pin 3 and 2 of arduino (least significant bit is 2) 
  // pin 3 = int1 pin 2 = int0
  EIMSK = 0b00000011;
  ////////////////32

  // interruzioni sul rising e falling edge 01, 11 solo rising
  // -- Google Translate IT > EN
  // breaks on the rising and falling edge 01, 11 only rising
  EICRA = 0b00001111;

  /* configurazione timer 2
   * quindi il valore della variabile putput e' compreso fra 0 e 625
   * -- Google Translate IT > EN --
   * timer 2 configuration so the value of the putput variable is between 0 and 625
   */
  TCCR2A = 0b00000011;
  TCCR2B = 0b00000001;
  TIMSK2 = 0b00000001;

#ifdef TEST_MODE
  // abilito le interruzioni del timer per la serial.available
  // Google Translate IT > EN --
  // enable timer interrupts for serial.available
  TIMSK0 = 0b00000001;
#else
  // disabilito interruzioni timer 0
  // Google Translate IT > EN --
  // disable timer interruptions 0
  TIMSK0 = 0b00000000;
#endif

  // settaggio della lettura analogica
  // setto il voltaggio di riferimento del convertitore alla tensione di
  // alimentazione e il multiplexer per la lettura
  // -- Google Translate IT > EN --
  // setting of the analogue reading with reference to the reference voltage
  // of the converter to the supply voltage and the multiplexer for reading
  ADMUX = (1 << 6) | (SPEED_READ & 0x07);

  // abilito l'ADC, le interruzioni hardware e il prescaler a 128 per letture
  // accurate su 10bit
  // -- Google Translate IT > EN --
  // enable the ADC, the hardware interruptions and the prescaler at 128 
  // for accurate readings on 10bit
  ADCSRA = 0b10001111;
  // non setto le lettura automatiche utilizzando il registro ADCSRB perche le
  // lancio a mano
  // -- Google Translate IT > EN --
  // I do not read automatic readings using the ADCSRB register because 
  // I launch them by hand

  Serial.begin(9600);

  zcd_tick_log = 0;
  tick_after_zcd = 0;

  uint16_t prog_output = 0;

  Serial.println(F("Calculating frequency ..."));
  tick_per_phase = calculate_main_power_frequency();

  Serial.print(F("Number of tick per half period: "));
  Serial.println(tick_per_phase);

  if (!my_digital_read(PIND, PROG_PIN)) {

    Serial.println(F("Programming mode"));

    // se il bottone e' pigiato chiededre di rilasciarlo
    // -- Google Translate IT > EN --
    // if the button is pressed, ask to release it
    check_programming_button();

    prog_output = (HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG;
    if (prog_output) {
      Serial.println(F("Turn speed potentiometer to lowest value ..."));
      while (prog_output) {
        prog_output = (HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG;
      }
    }

    // se il bottone e' pigiato chiededre di rilasciarlo
    // -- Google Translate IT > EN --
    // if the button is pressed, ask to release it
    check_programming_button();

    Serial.println(
        F("Turn potentiometer to increase speed and reach desire LOW speed, "
          "once done press and hold programming button ..."));

    button_hold_request(&loop_read_fun, &save_low_speed_exit_fun);

    // -- Google Translate IT > EN --
    // if the button is pressed, ask to release it
    check_programming_button();

    Serial.println(
        F("Turn potentiometer to increase speed and reach desire HIGH speed, "
          "once done press and hold programming button ..."));

    button_hold_request(&loop_read_fun, &save_high_speed_exit_fun);

    Serial.println(F("Configuration completed, enjoy!"));
    // impedisce la modalita' manuale quando si esce dal settaggio
    // -- Google Translate IT > EN --
    // prevents manual mode when exiting the setting
    check_programming_button();
  }
  Serial.println(F("Operating mode"));
  Serial.println(F("Reading paramters ..."));
  tacho_min_speed_value = (EEPROM.read(0) << 8) | EEPROM.read(1);
  output_min_speed_value = (EEPROM.read(2) << 8) | EEPROM.read(3);
  tacho_max_speed_value = (EEPROM.read(4) << 8) | EEPROM.read(5);
  output_max_speed_value = (EEPROM.read(6) << 8) | EEPROM.read(7);
  Serial.print(F("LOW speed tacho value: "));
  Serial.print(tacho_min_speed_value);
  Serial.print(F(", HIGH speed tacho value: "));
  Serial.println(tacho_max_speed_value);
  Serial.print(F("LOW speed output value: "));
  Serial.print(output_min_speed_value);
  Serial.print(F(", HIGH speed output value: "));
  Serial.println(output_max_speed_value);

#ifndef TEST_MODE
  init_params();
#endif

  output = output_min_speed_value;
  Output = output;

  motor_PID = new PID(&Input, &Output, &Setpoint, P_ON_M, &search,
                      &tacho_tick_log, DIRECT);

  // TODO quando sopra un certo livello azzerare il valore di output per rendere
  // il rallentamento migliore
  // -- Google Translate IT > EN --
  // TODO when above a certain level reset the output value to render
  // the best slowdown
  motor_PID->SetOutputLimits(
      0, output_min_speed_value +
             (abs(tick_per_phase - output_min_speed_value) >> 1));

  motor_PID->SetMode(AUTOMATIC);

#ifndef TEST_MODE
  //Serial.end();
#endif
}

volatile uint8_t _tacho_trig = 0;

void loop() {

  // valore del potenziometro
  // -- Google Translate IT > EN --
  // potentiometer value
  Setpoint = map(((HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG), 0, 1023,
                 tacho_min_speed_value, tacho_max_speed_value);

  if (my_digital_read(PIND, PROG_PIN)) {
    motor_PID->SetMode(AUTOMATIC);
    Input = tacho_tick_log;

    if (computeBarrier) {
      motor_PID->Compute();
      computeBarrier = 0;
    }

#ifdef TEST_MODE

    if (delay_counter - last_delay_counter > 60000) {

      if (Serial.available()) {
        serial_read = Serial.readString();
        String K = serial_read.substring(0, 2);

        if (K.equals("kp")) {
          k_param.Kp = serial_read.substring(2).toFloat();
        } else if (K.equals("ki")) {
          k_param.Ki = serial_read.substring(2).toFloat();
        } else if (K.equals("kd")) {
          k_param.Kd = serial_read.substring(2).toFloat();
        } else {
          Serial.print("invalid arguments");
        }

        Serial.print("Kp: ");
        Serial.print(k_param.Kp, 8);
        Serial.print(" Ki: ");
        Serial.print(k_param.Ki, 8);
        Serial.print(" Kd:");
        Serial.println(k_param.Kd, 8);
      }
      Serial.print("Setpoint: ");
      Serial.print(Setpoint);
      Serial.print(" Input: ");
      Serial.print(Input);
      Serial.print(" Output: ");
      Serial.println(Output);
      last_delay_counter = delay_counter;
    }
#endif

    output = Output;

  } else {
    motor_PID->SetMode(MANUAL);
    output = map(((HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG), 0, 1023,
                 tick_per_phase, 0);
  }

  /**
   * rendo il codice invulnerabile ai cambiamenti di frequenza della AC
   * in realta' ad essere precisi dovrei ricalcolare i valori limite di
   * velocita', ma per cambiamenti minimi (che dovrebbero essere la
   * regola) non e' necessario
   * -- Google Translate IT > EN --
   * by making the code invulnerable to changes in the frequency of the AC
   * in reality to be precise I should recalculate the speed limit values, 
   * but for minimal changes (which should be the rule) it is not necessary
   * */
  tick_per_phase = zcd_tick_log;

  /**
   * zcd circuit fault: in caso di fault dello zcd disabilito il triac e
   * imposto l'output al massimo valore, equivale a motore spento
   * questa cosa puo' essere fatta anche nel isr, valutare se lasciare qui o
   * spostare
   * -- Google Translate IT > EN --
   * zcd circuit fault: in the event of a zcd fault, disabling the triac and
   * setting the output to the maximum value, it is equivalent to the engine
   * off, this thing can also be done in the isr, evaluate whether to leave 
   * here or move
   * */
  if (tick_after_zcd > ZCD_FAIL_LIMIT) {
    output = 65535;
    TURN_OFF_TRIAC();
  }

  /**
   * rilevamento blocco del motore
   * -- Google Translate IT > EN --
   * engine block detection
   * */
  if (tick_after_tacho > TACHO_FAIL_LIMIT) {
    //do something
    //Serial.println("hang detection");
    /*output = 65535;
    TURN_OFF_TRIAC();*/
  }


}

// handler dell' interrupt associato al pin 2 di arduino
// utilizzato per la sincronizzazione di fase, e' connesso
// all'uscita del circuito di ZCD
// -- Google Translate IT > EN --
// interrupt handler associated with arduino pin 2 used 
// for phase synchronization, connected to the ZCD circuit output
ISR(INT0_vect) {
  zcd_error_correction = 0;
  found_correct_main_phase = 0;
  // lancia una lettura del valore analogico
  // -- Google Translate IT > EN --
  // launches an analog value reading
  ANALOG_READ();
}

// handler dell' interrupt associato al pin 3 di arduino
// utilizzato per il rilevamento del tacogeneratore
// -- Google Translate IT > EN --
// interrupt handler associated with pin 3 of arduino used for 
// the detection of the tacogenerator
ISR(INT1_vect) {
  tacho_error_correction = 0;
  found_correct_tacho_phase = 0;
}

// interrupt associato al timer, all'interno di una semionda della rete
// effettua dei controlli periodici, quali l'incremento del contatore
// per poter far si che il bottone do programmazione funzioni c'e' bisogno dello
// zcd
// -- Google Translate IT > EN --
// interrupt associated with the timer, inside a network half-wave it carries 
// out periodic checks, such as the counter increment to be able to make the 
// programming button work, there is a need for the zcd
ISR(TIMER2_OVF_vect) {
  //
  // zcd
  //
  if (my_digital_read(PIND, ZCD_INPUT)) {
    zcd_error_correction++;
  }

  if (zcd_error_correction > 10 && !found_correct_main_phase) {
    zcd_tick_log = tick_after_zcd;
    frequency_calc_added = 1;
    if (!my_digital_read(PIND, PROG_PIN) && !delay_allow) {
      delay_count++;
    } else {
      delay_count = 1;
      delay_allow = 0;
    }
    if (!delay_count) {
      delay_allow = 1;
    }
    tick_after_zcd = 0;
    // spegni il triac
    TURN_OFF_TRIAC();
    triac_state = 0;
    found_correct_main_phase = 1;
  }
  //
  // tacho
  //
  if (my_digital_read(PIND, TACHO_INPUT)) {
    tacho_error_correction++;
  }

  if (tacho_error_correction > 15 && !found_correct_tacho_phase) {
    tacho_tick_log = tick_after_tacho;
    computeBarrier = 1;
    tick_after_tacho = 0;
    found_correct_tacho_phase = 1;

#ifdef TEST_MODE
    _tacho_trig = !_tacho_trig;
    if (_tacho_trig) {
      TURN_ON_TACHO_LOG();
    } else {
      TURN_OFF_TACHO_LOG();
    }
#endif
  }
  //////////////////////////////////////
  tick_after_zcd++;
  tick_after_tacho++;

  delay_counter++;

  if (tick_after_zcd >= output && !triac_state) {
    // accendi il triac
    // -- Google Translate IT > EN --
    // turn on the triac
    TURN_ON_TRIAC();
    triac_state = 1;
  }
}

// al completamento della lettura del valore del potenziometro salva i valori
// -- Google Translate IT > EN --
// on completion of reading the value of the potentiometer saves the values
ISR(ADC_vect) {
  LOW_ANALOG_REG = ADCL;
  HIGH_ANALOG_REG = ADCH;
}
