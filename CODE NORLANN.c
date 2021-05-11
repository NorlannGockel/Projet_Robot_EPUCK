 /****************************************************************************************
 * File:   Projet_E-puck.c
 * Date:   Decembre 2020     
 * Author: C.SABOURIN  
 DEPUYDT Célia BEHUET Matthieu

 * Description: Programme permettant d'iniailiser le controle du robot E-puck 
 
    
 * Modifications: 
 
 Ressouces:
 https://cyberbotics.com/#webots
 https://cyberbotics.com/doc/guide/epuck
 https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers
 ****************************************************************************************/ 
 
 
 


/********************************************************************************************/
/********************************************************************************************/
/*   Inclusion headers langace C                                                            */                                               
/********************************************************************************************/
/********************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webots/device.h>

/********************************************************************************************/
/********************************************************************************************/
/*   Inclusion headers files Webots  (20201214                                              */                                               
/********************************************************************************************/
/*****************************************************/
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>
#include <webots/keyboard.h>

/********************************************************************************************/
/********************************************************************************************/
/*   Définition des constantes                                                              */                                               
/********************************************************************************************/
/********************************************************************************************/
#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 6.28
#define DISTANCE_SENSORS_NUMBER 8

#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)

#define PI 3.14159265358979


/********************************************************************************************/
/********************************************************************************************/
/*   Déclaration des variables globales                                                     */                                               
/********************************************************************************************/
/********************************************************************************************/
static double speeds[2];

// Pointer file
FILE* fichier = NULL;
/********************************************************************************************/
/********************************************************************************************/


/********************************************************************************************/
/********************************************************************************************/
/*   Déclaration et définitions des actionneurs et capteurs du robot                        */
/********************************************************************************************/
/********************************************************************************************/

/**************************************************************************/
/*   Déclaration et définition des capteurs  de proximités de proximités  */
/*    - distance_sensors_values: tableau des mesures capteurs             */ 
/**************************************************************************/
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double distance_sensors_values[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

/**************************************************************************/
/*   Déclaration et définition des capteurs des leds led0, .. led7        */
/*   - leds_values: tableau de l'état des leds                            */ 
/**************************************************************************/
#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
static bool leds_values[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};

/**************************************************************************/
/*   Déclaration et définition des des deux moteurs                       */
/*   - left_motor: variable vitesse moteur gauche                         */ 
/*   - right_motor: variable vitesse moteur droit                         */ 
/*   - left_position_sensor: variable codeur moteur gauche                */ 
/*   - right_position_sensor: variable codeur moteur droit                */ 
/**************************************************************************/
static WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;

/**************************************************************************/
/*   Déclaration et définition de la caméra du robot                      */
/*   - *image: pixels de l'images                                         */ 
/*   - image_width: largeur image                                         */ 
/*   - image_height: hauteur image                                        */ 
/**************************************************************************/
static WbDeviceTag cam;
const unsigned char *image;
unsigned short image_width, image_height;


/********************************************************************************************/
/********************************************************************************************/
/*  Déclaration prototype fonction programme robot Epuck                                    */ 
/********************************************************************************************/
/********************************************************************************************/

static double compute_angle(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor);

/**************************************************************************/
/*  Fonction retourant le temps de simulation                             */ 
/**************************************************************************/
static int get_time_step(void);

/**************************************************************************/
/*  Fonction "pas de temps" simulation                                    */ 
/**************************************************************************/
static void step();

/**************************************************************************/
/*  Fonction temporisation                                                */ 
/**************************************************************************/
static void passive_wait(double sec);




/**************************************************************************/
/*  Initialisation du robot Epuck                                         */ 
/*  - Initialisation leds                                                 */
/*  - Initialisation des capteurs de proximité                            */
/*  - Initialisation des moteurs                                          */
/*  - Initialisation des codeurs roues                                    */
/*  - Initialisation de la camera                                         */
/**************************************************************************/
static void init_epuck();

/**************************************************************************/
/*  Fonction  get_sensor_input:                                           */ 
/*  Permet de récupérer les valeurs des capteurs ps0 à ps7                */
/**************************************************************************/
static void get_sensor_input();

/**************************************************************************/
/*  Fonction set_actuators     :                                          */ 
/*  Permet de faire la mise à jour des vitesses des roues du robot        */
/**************************************************************************/
static void set_speed_motors(double speed_left, double speed_right);

/**************************************************************************/
/*  Fonction reset_speed_values:                                          */ 
/*  Permet de remettre à zero les vitesses des deux moteurs                */
/**************************************************************************/
static void reset_speed_motors();

/**************************************************************************/
/*  Fonction set_leds :                                                   */ 
/*  Permet de faire la mise à jour de l'état des leds du robot            */
/**************************************************************************/
static void set_leds();

/**************************************************************************/
/*  Fonction reset_led_values:                                            */ 
/*  Permet d'éteindre toutes les lesds du robot                           */
/**************************************************************************/
static void reset_leds();

/**************************************************************************/
/*  Fonction blink_leds                                                   */ 
/*  Permet de faire clignoter des leds                                    */
/**************************************************************************/
static void blink_leds();

/**************************************************************************/
/*  Fonction covered_distance                                             */ 
/*  Permet de calculer la distance parcourue par le robot                 */
/**************************************************************************/
static double covered_distance(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor);

/**************************************************************************/
/*  Fonction covered_distance                                             */ 
/*  Permet de calculer l'angle de rotation                               */
/**************************************************************************/
static double covered_angle(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor);


/********************************************************************************************/
/********************************************************************************************/
/* Prototype des fonctions à coder:                                                         */ 
/********************************************************************************************/
/********************************************************************************************/

/**************************************************************************/
/*  Fonction demi_tour_gauche                                              */ 
/*  Permet au robot de faire un demi tour vers la gauche                  */
/**************************************************************************/
static void demi_tour_gauche();

/**************************************************************************/
/*  Fonction quart_tour_gauche                                            */ 
/*  Permet au robot de faire un quart de tour vers la droite              */
/**************************************************************************/
static void quart_tour_gauche();

/**************************************************************************/
/*  Fonction demi_tour_droit                                              */ 
/*  Permet au robot de faire un demi tour vers la droite                  */
/**************************************************************************/
static void demi_tour_droit();

/**************************************************************************/
/*  Fonction quart_tour_droit                                             */ 
/*  Permet au robot de faire un quart de tour vers la droite              */
/**************************************************************************/
static void quart_tour_droit();



/**************************************************************************/
/*  Fonction detection d'obstacle                                         */ 
/*  Permet au robot de détecter les obstacles                             */
/*                   0 si pas obstacle                                    */
/*                   1 si obstacle à droite                               */
/*                   2 si obstacle à gauche                               */
/**************************************************************************/

static int detection_obstacle(double sensors_value[8]);

/**************************************************************************/
/*  Fonction robot_avance_distance                                        */ 
/*  Permet au ronot d'avancer d'une ceratine distance                     */
/**************************************************************************/

static void robot_avance_distance(double distance_a_parcourrir);

/**************************************************************************/
/*  Fonction robot_evitement_obstacle()                                   */ 
/*  Permet au ronot d'éviter les obstacles                                */
/**************************************************************************/
static void robot_evitement_obstacle();

/**************************************************************************/
/*  Fonction control_manuel_epuck                                         */ 
/*  Permet de controler le robot avec le calvier                          */
/*    - A : marche avant                                                  */
/*    - R : marche arrière                                                */
/*    - D : rotation droite                                               */
/*    - G : rotation gauche                                               */
/*    - L : clignotement leds                                             */
/*    - S : affiche l'état des capteurs                                   */
/**************************************************************************/
static void control_manuel_epuck();

// Fontion permettant de faire un carré 

static void faire_un_carre(double distance_a_parcourrir);

// Fonction permettant de choisir le type d'obstacle devant le robot

static void type_obstacle (); 




/********************************************************************************************/
/********************************************************************************************/
/*  Programme principal:                                                                    */ 
/********************************************************************************************/
/********************************************************************************************/
int main(int argc, char **argv)  {
double debut_time=0;

double pixel_r=0, pixel_g=0, pixel_b=0;
double  pixel_l=0,  pixel_theta=0,  pixel_phi=0;
int x=0,y=0, i=0;
int presence_obstacle;
int presence_balle=0;
int position_balle=0;



  wb_robot_init();
 
 

  
  // initialisation du robot
  init_epuck();
  
  
 /* //Fontion permettant de faire un carré 

  faire_un_carre(0.2);
 
 */ 
 
  
 /* // avancer d'un distance indiquée 
  robot_avance_distance(0.5);
  
  */
  
  
       
  
  
  // contôle manuel du robot
  control_manuel_epuck();
  
  
 
  

   
  while (true)  {
        
     printf("time simulation =%f \n",wb_robot_get_time());
     
     //lectures des données capteurs
     get_sensor_input();
     
     
    int HistoW[image_width]; 
    int maximum_histo=0; 
    int minimum_histo=0;
    double sensors_value[8];
    
    
    
     
   // Evitment d'obstacle 
     
      
       if (presence_balle==0) {  // Un obstacle est detecté mais ce n'est ps la balle ici, balle=0
            
               
/**************************************************************************/
/*  Fonction detection d'obstacle                                         */ 
/*  Permet au robot de détecter les obstacles                             */
/*                                                                        */
/*              1 si obstacle à droite                                    */
/*              2 si obstacle à gauche                                    */
/**************************************************************************/
               
                presence_obstacle=detection_obstacle(distance_sensors_values);
                  if (presence_obstacle == 1){
                        reset_leds();
                        leds_values[7] = true;
                        leds_values[6] = true;
                        set_leds();
                        quart_tour_gauche();
                        reset_leds();
                   }
                       
                  if (presence_obstacle == 2){
                       reset_leds();
                       leds_values[1] = true;
                       leds_values[2] = true;
                       set_leds(); 
                       quart_tour_droit();
                       reset_leds();
                  }   
                  
        }       
               
   
   // La Balle est détectée            
              
    
            
              image = wb_camera_get_image(cam);
              presence_balle=0;   


       // lectures pixel camera
  
      for (x = 0; x < image_width; x++) {
         HistoW[x]=0;

          for (y = 0; y < image_height; y++) {
            pixel_r = wb_camera_image_get_red(image, image_width, x, y);
            pixel_g = wb_camera_image_get_green(image, image_width, x, y);
            pixel_b = wb_camera_image_get_blue(image, image_width, x, y);
  
            pixel_l=sqrt(pixel_r*pixel_r+pixel_g*pixel_g+pixel_b*pixel_b);
            pixel_phi=atan(pixel_g/pixel_r);
            pixel_theta=sqrt(pixel_r*pixel_r+pixel_g*pixel_g)/pixel_l;

            if((pixel_phi>0.9) && (pixel_theta>0.9) ) { HistoW[x]=HistoW[x]+1;}
          }
    
        if ( HistoW[x]>5) presence_balle=1;
      }
 
      
      
      if (presence_balle==1) {
      maximum_histo=0;
      position_balle=0;
      
       printf("Balle detectée \n");
       printf ("l= %.2f \n",  pixel_r );
       printf ("phi= %.2f \n",  pixel_phi );
       printf ("theta= %.2f \n",  pixel_theta );
       
 
       
       for (i=0; i<image_width; i++) {
           if (HistoW[i]>HistoW[maximum_histo])  // Avec ces commandes on peut connaître le maximum du tableau
           {
            position_balle=i;
          
            }
            
           maximum_histo=HistoW[i];

           } 
            
       } 
       
    
     
  
  // Ici la balle est detéctée   
    
       if (presence_balle==1) {  
          
          set_speed_motors(3,3); 
          printf("position_balle = %d", position_balle);
          
            if (position_balle < 23 ) // Si la balle se trouve sur la gauche le robot tournera a gauche
              set_speed_motors(-2,2);
             
              
            if (position_balle > 29 ) // Si la balle se trouve sur la droite le robot tournera a droite
              set_speed_motors(2,-2);
              
              
            if (position_balle > 23 && position_balle < 29) // Si la balle se trouve au milieu du champ de vision du robot, il avancera
              set_speed_motors(3,3);
              
      
       if  (distance_sensors_values[0] > 105) {  // Ces commandes servent à s'arrêter devant la balle, tourner sur lui-même et faire clignoter les leds
          set_speed_motors(0,0);
          blink_leds();   
          set_speed_motors(-2,2);
          passive_wait(5);
          set_speed_motors(0,0);
          passive_wait(5);
          reset_leds();
          
          
       }
       
       }
       
      
      
      
       if (presence_balle==0) {     //Ici aucune balle n'est detéctée, ducoup il avancera
           set_speed_motors(2,2);
      
       } 
       
     
       

       
     
     //clignotement leds
    // blink_leds();   
      
    
     // set vitesse vitesses moteur   
    //   set_speed_motors(2,2);
     
     
     
         
    // Pas de temps simulation    
     step();
         
         
   }
   
    return EXIT_SUCCESS;
}



/********************************************************************************************/
/********************************************************************************************/
/*  Fonction programme robot Epuck (20201214)                                               */ 
/*                                                                                          */
/********************************************************************************************/
/********************************************************************************************/


/**************************************************************************/
/*  Fonction retourant le temps de simulation                             */ 
/**************************************************************************/
static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

/**************************************************************************/
/*  Fonction "pas de temps" simulation                                    */ 
/**************************************************************************/
static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

/**************************************************************************/
/*  Fonction temporisation                                                */ 
/**************************************************************************/
static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

/**************************************************************************/
/*  Initialisation du robot Epuck                                         */ 
/*  - Initialisation leds                                                 */
/*  - Initialisation des capteurs de proximité                            */
/*  - Initialisation des moteurs                                          */
/*  - Initialisation des codeurs roues                                    */
/*  - Initialisation de la camera                                         */
/**************************************************************************/
static void init_epuck() {
  int i;
  
  printf("---Lancementinitialisation Epuck-------------\n");
  
   // Initialisation leds 
  printf("---------Initialisation leds------------------\n");
  for (i = 0; i < LEDS_NUMBER; i++)
    leds[i] = wb_robot_get_device(leds_names[i]);
  
    
   // Initialisation des capteurs de proximité
  printf("---------Initialisation des capteurs de proximité---\n"); 
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], get_time_step());
  }
  
   // Initialisation des moteurs  
  printf("---------Initialisationm des moteurs-----------------\n"); 
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
   // Initialisation des codeurs roues  
  printf("---------Initialisation des codeurs roues-------\n"); 
  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, 32);
  wb_position_sensor_enable(right_position_sensor, get_time_step());
  
   // Initialisation de la camera  
  printf("---------Initialisation de la camera------------------\n"); 
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, get_time_step());
  image_width = wb_camera_get_width(cam);
  image_height = wb_camera_get_height(cam);

 
  //initialisation du clavier
 
  wb_keyboard_enable(get_time_step());
  
  step();
  
  printf("---Lancementinitialisation Epuck-------------\n");
  
}

/**************************************************************************/
/*  Fonction  get_sensor_input:                                           */ 
/*  Permet de récupérer les valeurs des capteurs ps0 à ps7                */
/**************************************************************************/
static void get_sensor_input() {
  int i;
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);
   }
}

/**************************************************************************/
/*  Fonction set_actuators     :                                          */ 
/*  Permet de faire la mise à jour des vitesses des roues du robot        */
/**************************************************************************/
static void set_speed_motors(double speed_left, double speed_right) {
  wb_motor_set_velocity(left_motor, speed_left);
  wb_motor_set_velocity(right_motor, speed_right);
}


/**************************************************************************/
/*  Fonction reset_speed_values:                                          */ 
/*  Permetde remettre à zero les vitesses des deux moteurs                */
/**************************************************************************/

static void reset_speed_motors() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);  
}



/**************************************************************************/
/*  Fonction set_leds :                                                   */ 
/*  Permet de faire la mise à jour de l'état des leds du robot            */
/**************************************************************************/
static void set_leds() {
  int i;
  for (i = 0; i < LEDS_NUMBER; i++)
    wb_led_set(leds[i], leds_values[i]);
}

/**************************************************************************/
/*  Fonction reset_led_values:                                            */ 
/*  Permet d'éteindre toutes les lesds du robot                           */
/**************************************************************************/
static void reset_leds() {
  int i;
  for (i = 0; i < LEDS_NUMBER; i++)
    wb_led_set(leds[i], 0);  
}


/**************************************************************************/
/*  Fonction blink_leds                                                   */ 
/*  Permet de faire clignoter des leds                                    */
/**************************************************************************/
static void blink_leds() {
  static int counter = 0;
  int i;
  counter++;
  for (i = 0; i < LEDS_NUMBER; i++) leds_values[i]= false;
  leds_values[(counter / 10) % LEDS_NUMBER] = true;
  set_leds();
}


/**************************************************************************/
/*  Fonction covered_distance                                             */ 
/*  Permet de calculer la distance parcourue par le robot                 */
/**************************************************************************/
static double covered_distance(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r* WHEEL_RADIUS;         // distance covered by right wheel in meter
  double distance = (dr + dl) / 2;  // ddistance covered by robot
  //printf("distance= %f \n", distance);
  return distance;
}


/**************************************************************************/
/*  Fonction covered_angle                                                */ 
/*  Permet de calculer la distance parcourue par le robot                 */
/**************************************************************************/
static double covered_angle(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r* WHEEL_RADIUS;         // distance covered by right wheel in meter
  double angle = (dr - dl) / AXLE_LENGTH;  // ddistance covered by robot
  //printf("angle= %f \n", angle);
  return angle;
}

/*  Fonction demi_tour_gauche                                              */ 
/*  Permet au robot de faire un demi tour vers la gauche                  */
/**************************************************************************/
    static void demi_tour_gauche(){
      set_speed_motors(-2,2);
      passive_wait(2.5);
  

   }

/**************************************************************************/
/*  Fonction quart_tour_gauche                                            */ 
/*  Permet au robot de faire un quart de tour vers la droite              */
/**************************************************************************/
   static void quart_tour_gauche(){
      set_speed_motors(-2,2);
      passive_wait(1.2);
 
   }
/**************************************************************************/
/*  Fonction demi_tour_droit                                              */ 
/*  Permet au robot de faire un demi tour vers la droite                  */
/**************************************************************************/
   static void demi_tour_droit(){
      set_speed_motors(2,-2);
      passive_wait(2.5);
 
   }
/**************************************************************************/
/*  Fonction quart_tour_droit                                             */ 
/*  Permet au robot de faire un quart de tour vers la droite              */
/**************************************************************************/
    static void quart_tour_droit(){
       set_speed_motors(2,-2);
       passive_wait(1.2);
 
    }



/**************************************************************************/
/*  Fonction detection d'obstacle                                         */ 
/*  Permet au robot de détecter les obstacles                             */
/*              0 si pas obstacle                                         */
/*              1 si obstacle à droite                                    */
/*              2 si obstacle à gauche                                    */

   static int detection_obstacle(double sensors_value[8]){

   int obstacle = 0;

    
      
      if (sensors_value[0] > 150 || sensors_value[1] > 120) {
      obstacle = 1;
  
    }
     
     if (sensors_value[6] > 120 || sensors_value[7] > 150) {
     obstacle=2; 
  
     }
     
  return obstacle;
  }


/**************************************************************************/
/*  Fonction control_manuel_epuck                                         */ 
/*  Permet de controler le robot avec le calvier                          */
/*    - A : marche avant                                                  */
/*    - R : marche arrière                                                */
/*    - D : rotation droite                                               */
/*    - G : rotation gauche                                               */
/*    - L : clignotement leds                                             */
/*    - S : affiche l'état des capteurs                                   */
/**************************************************************************/
static void control_manuel_epuck() {
int i = 0;
int key = 0;

    printf ("Appuyer sur A pour avancer\n");
    printf ("Appuyer sur R pour reculer\n");
    printf ("Appuyer sur D pour tourner à droite\n");
    printf ("Appuyer sur G pour tourner à gauche\n");
    printf ("Appuyer sur L pour faire clignoter les leds \n");
    printf ("Appuyer sur S pour afficher l'état des capteurs \n");
    printf ("Appuyer sur C pour continuer... \n");
    
    
    do{
      key=wb_keyboard_get_key(); 
      set_speed_motors(0,0);
      
      if (key=='A'){
      set_speed_motors(2,2);
      }
      
        if (key=='R'){
      set_speed_motors(-2,-2);
      }
      
      if (key=='D'){
      set_speed_motors(2,-2);
      }
      
      if (key=='G'){
      set_speed_motors(-2,2);
      }
      
      if (key=='L'){  
      blink_leds();
      }
      
       if (key=='S'){
       get_sensor_input();
      printf ("ps0=%f \n" ,distance_sensors_values[0]);
      printf ("ps1=%f \n" ,distance_sensors_values[1]);
      printf ("ps2=%f \n" ,distance_sensors_values[2]);
      printf ("ps3=%f \n" ,distance_sensors_values[3]);
      printf ("ps4=%f \n" ,distance_sensors_values[4]);
      printf ("ps5=%f \n" ,distance_sensors_values[5]);
      printf ("ps7=%f \n" ,distance_sensors_values[7]);
      printf ("ps8=%f \n" ,distance_sensors_values[8]);
      }
      
         step();
       } while(key!='C') ;
          
  }    
  
/**************************************************************************/
/*  Fonction robot_avance_distance                                        */ 
/*  Permet au robot d'avancer d'une certaine distance                     */
/**************************************************************************/

  static void robot_avance_distance(double distance_a_parcourrir){

  double distance_initiale=0;
  


   distance_initiale = covered_distance(left_position_sensor, right_position_sensor);



    while (covered_distance(left_position_sensor, right_position_sensor) < (distance_initiale+distance_a_parcourrir) ){
    
     
     set_speed_motors(2,2);
     step ();
     }
     set_speed_motors(0,0);
}
     
     
  // Fontion permettant de faire un carré 

    void faire_un_carre(double distance_a_parcourrir) {
    int i=0;

       for (i=0; i<4; i++)
       {
         robot_avance_distance(distance_a_parcourrir);
         quart_tour_gauche();
       }
    return 0;
    
    }
    
    

   
