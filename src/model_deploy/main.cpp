#include "mbed.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

#include "accelerometer_handler.h"
#include "stm32l475e_iot01_accelero.h"
#include "config.h"
#include "magic_wand_model_data.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "uLCD_4DGL.h"
using namespace std::chrono;

#include "mbed_rpc.h"

EventQueue queue(32 * EVENTS_EVENT_SIZE);

#define pi 3.1415926

////----------------RPC-----------------------////
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
BufferedSerial pc(USBTX, USBRX);
void gesture(Arguments *in, Reply *out);
void analyse(Arguments *in, Reply *out);
void resewee(Arguments *in, Reply *out);
void angle(Arguments *in, Reply *out);
RPCFunction rpcresewee(&resewee, "resewee");
RPCFunction rpcgesture(&gesture, "gesture");
RPCFunction rpcanalyse(&analyse, "analyse");
RPCFunction rpcangle(&angle, "angle");

//double x, y;
int mode, deg, MODE, deg_s;
int flag = 0;
int P = 1;            // to stop detecting gesture
int num = 1;          // over times
double angle_current;
int16_t XYZ[3] = {0};
int16_t ini;
int D;
int N = 1;
int ges, guess;
int m, M;
int rst = 1;
int x, y, z;

Thread MQTTT;
Thread gesturet;
Thread t;
Thread pubThread;

EventQueue pubqueue;

uLCD_4DGL uLCD(D1, D0, D2); // serial tx, serial rx, reset pin;
InterruptIn confirm(USER_BUTTON);

//extern "C" void mbed_reset();

void resewee (Arguments *in, Reply *out)  {
   //rst = 0;
   NVIC_SystemReset();
}



////-----------------MQTT part----------------////

// GLOBAL VARIABLES
WiFiInterface *wifi;
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
const char* topic = "Mbed";
Thread mqtt_thread(osPriorityHigh);
EventQueue mqtt_queue;
void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    //P = 0;    // stop detecting gesture
    //message_num++;
    MQTT::Message message;
    char buff[100];
    if (MODE == 0){
      BSP_ACCELERO_Init();
      BSP_ACCELERO_AccGetXYZ(XYZ);
      sprintf(buff, "MODE = %d, gesture: %d, num = %d, V = %5d, %5d, %5d", MODE, M, N, XYZ[0], XYZ[1], XYZ[2]);
      printf("%d\n", M);
      N++;
      if(N == 5) P = 0;
      //flag = 1;
      uLCD.locate(1,2);
      uLCD.color(GREEN);
      uLCD.printf("%d", ges);
    }else{
      sprintf(buff, "MODE = %d, guess = %d\n" , MODE, guess);
      num++;
    }
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);
    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
    
}
void close_mqtt() {
    closed = true;
}
void MQTT_thread() {
    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return ;
    }
    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return ;
    }
    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    //TODO: revise host to your IP
    const char* host = "172.20.10.11";
    printf("Connecting to TCP network...\r\n");
    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);
    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting
    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return ;
    }
    printf("Successfully connected!\r\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";
    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }
    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    confirm.rise(mqtt_queue.event(&publish_message, &client));
    //btn2.rise(mqtt_queue.event(&publish_message, &client));
    //btn3.rise(&close_mqtt);
    int num = 0;
    while (num != 5) {
            client.yield(100);
            ++num;
    }
    while (1) {
            if (closed) break;
            client.yield(500);
            ThisThread::sleep_for(500ms);
    }
    printf("Ready to close MQTT Network......\n");
    if ((rc = client.unsubscribe(topic)) != 0) {
            printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
    printf("Failed: rc from disconnect was %d\n", rc);
    }
    mqttNetwork.disconnect();
    printf("Successfully closed!\n");
    return ;
}

////-----------------gesture part----------------////
Thread thread;

      // gesture index

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// Return the result of the last prediction
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}


void gesture_thread() {

  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return ;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return ;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return ;
  }

  error_reporter->Report("Set up successful...\n");
  while (true) {

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);
    m = gesture_index;
    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // Produce an output
    if (gesture_index < label_num) {
      error_reporter->Report(config.output_message[gesture_index]);
    }
    if(P == 0){
      return;
    }
  }
}

void gesture (Arguments *in, Reply *out)  
 {
   
   MQTTT.start(MQTT_thread);

     gesturet.start(gesture_thread);
     MODE = 0; // UI mode

   /*Thread t;//(osPriorityHigh);
   t.start(callback(&queue, &EventQueue::dispatch_forever));
   confirm.rise(queue.event(confirmangle));*/
////////////////*********turn on orange LED******************////
   led3 = 1;
////////////////////

      uLCD.color(GREEN);
      uLCD.locate(1,2);
        
      while(1){
        if(P == 0){
          return;
        }
        if(m == 0) 
        {
            M = m;
            uLCD.locate(1,2);
            uLCD.printf("%d", m);//}
        }
        if(m == 1) 
        {
            M = m;
            uLCD.locate(1,2);
            uLCD.printf("%d", m);//}
        }
        if(m == 2)
        {
            M = m;
            uLCD.locate(1,2);
            uLCD.printf("%d", m);
            //}
        }
        ThisThread::sleep_for(50ms);
     }
        ThisThread::sleep_for(50ms);
 }



////-----------------tilt part----------------////
void angle (Arguments *in, Reply *out)  
 {
    x = in->getArg<int>();
    y = in->getArg<int>();
    z = in->getArg<int>();
 }

void analyse (Arguments *in, Reply *out)  
 {
   ////////connect to wifi///////////
    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return ;
    }
    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return ;
    }
    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    //TODO: revise host to your IP
    const char* host = "172.20.10.11";
    printf("Connecting to TCP network...\r\n");
    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);
    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting
    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return ;
    }
    printf("Successfully connected!\r\n");
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";
    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }
    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));

    //////////////////////
    MODE = 1;
  ////////////////*********turn on orange LED******************////
    led3 = 0;

    printf("start analysize");
    if(y > 0) guess = 2;
    else{
      if(x<0) guess = 1;
      else guess = 0;
    }

    pubqueue.call(&publish_message, &client);

    ThisThread::sleep_for(500ms);
 }


int main()
{
      uLCD.text_width(4); //4X size text
      uLCD.text_height(4);
      pubThread.start(callback(&pubqueue, &EventQueue::dispatch_forever));

   //The mbed RPC classes are now wrapped to create an RPC enabled version - see RpcClasses.h so don't add to base class
   
    // receive commands, and send back the responses
    char buf[256], outbuf[256];

    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    while(1) {
        memset(buf, 0, 256);
        for (int i = 0; ; i++) {
            char recv = fgetc(devin);
            if (recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
        
    }
}


