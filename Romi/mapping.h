#ifndef _Mapping_h
#define _Mapping_h
#include <EEPROM.h>

const byte MAP_RESOLUTION = 25;
const byte MAP_DEFAULT_FEATURE = '#';
const byte MAP_EXPLORED_FEATURE = '.';
const int MAP_X=1440;
const int MAP_Y=1440;

class Mapper
{
    public:
        void resetMap();
        void printMap();
        void convolute(float robot_x,float robot_y);
        void updateMapFeature(byte feature, int y, int x);
        void updateMapFeature(byte feature, float y, float x);
        char readCell(float y, float x);
        int  indexToPose(int i, int map_size, int resolution);
        int  poseToIndex(int x, int map_size, int resolution);
        float percent();
        float goal_area_x;
        float goal_area_y;
    private:
        int X_size;
        int Y_size;
};

void Mapper::resetMap()
{

    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for (int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            
            if (eeprom_address > 1023)
            {
                Serial.println(F("Error: EEPROM Address greater than 1023"));
            }
            else
            {
                EEPROM.update(eeprom_address, MAP_DEFAULT_FEATURE );
                
            }
        }
    }

}

char Mapper::readCell(float y, float x){

  int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
  int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);  
  int eeprom_address = (x_index*MAP_RESOLUTION)+y_index;
  byte value;
  value = EEPROM.read(eeprom_address);//, value);

  return char(value);
}

void Mapper::printMap()
{

    Serial.println("Map");
    for (int y_index=MAP_RESOLUTION-1;y_index>=0;y_index--)
    {
        for(int x_index=0;x_index<MAP_RESOLUTION;x_index++){
          
            int eeprom_address = (x_index*MAP_RESOLUTION)+y_index;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            Serial.print( (char)value );
            Serial.print("  ");
        }
        Serial.println("");
    }
  
}
float Mapper::percent()
{
   int explored_cells = 0;
   // Serial.println("Map");
    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for(int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            if ((char)value != MAP_DEFAULT_FEATURE)
            {
              explored_cells = explored_cells+1;
            }
        }
        //Serial.println("");
    }
  float percent = float(explored_cells)/float(MAP_RESOLUTION*MAP_RESOLUTION);
  Serial.print("percent explored: ");
  Serial.println(percent);
  return percent;
}

int Mapper::poseToIndex(int x, int map_size, int resolution)
{
    //1800/ (1800/25)
    return x / (map_size / resolution);
}

int Mapper::indexToPose(int i, int map_size, int resolution)
{
    return i* (map_size / resolution);
}

void Mapper::convolute(float robot_x,float robot_y)
{
    char processed_map[MAP_RESOLUTION][MAP_RESOLUTION];

    for (int i =0;i<MAP_RESOLUTION;i++){
          for (int j =0;j<MAP_RESOLUTION;j++){
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);
            if (value == MAP_DEFAULT_FEATURE){
            processed_map[j][i] = 1;}
            else
            {processed_map[j][i] = 0;}
          }
    }
    char max_score = 0;
    float best_dist = 0;
    char processed_map2[MAP_RESOLUTION][MAP_RESOLUTION];
    for (int i =0;i<MAP_RESOLUTION;i++){
      for (int j =0;j<MAP_RESOLUTION;j++){
        char sum = 0;
         for (int i2 =i-3;i2<=i+3;i2++){
            for (int j2 =j-3;j2<=j+3;j2++){
              if ((i2>=0 && i2 < MAP_RESOLUTION)&&(j2>=0 && j2 < MAP_RESOLUTION)){ 
              sum = sum + processed_map[j2][i2];}
            }
         }
         bool set_goal = false;
          float e_x = (robot_x-indexToPose(i,MAP_X,MAP_RESOLUTION));
          float e_y = (robot_y-indexToPose(i,MAP_Y,MAP_RESOLUTION));
          e_x*=e_x;
          e_y*=e_y;
          
          float dist = sqrt(e_x+e_y);
          if (sum>max_score ){
            set_goal = true;
          }
          else if (max_score == sum){
          
            if (dist < best_dist)
            {
              set_goal = true;
            }
            }
          if (set_goal){
            max_score = sum;
            best_dist = dist;
            goal_area_x = indexToPose(i,MAP_X,MAP_RESOLUTION)+MAP_X/(2*MAP_RESOLUTION);
            goal_area_y = indexToPose(j,MAP_Y,MAP_RESOLUTION)+MAP_Y/(2*MAP_RESOLUTION);
            Serial.print("g_x: "); Serial.print(goal_area_x); Serial.print("g_y: "); Serial.println(goal_area_y);
          }
         processed_map2[j][i]= sum;
      }
    }

    for (int y_index=MAP_RESOLUTION-1;y_index>=0;y_index--)
    {
        for(int x_index=0;x_index<MAP_RESOLUTION;x_index++){
          
            int eeprom_address = (x_index*MAP_RESOLUTION)+y_index;
            byte value;
            //value = EEPROM.read(eeprom_address);//, value);
            Serial.print( (int)processed_map[y_index][x_index] );
            Serial.print("  ");
        }
        Serial.println("");
    }
    int pos_x_index = poseToIndex(robot_x, MAP_X, MAP_RESOLUTION);
    int pos_y_index = poseToIndex(robot_y, MAP_Y, MAP_RESOLUTION);  
    int goal_x_index = poseToIndex(goal_area_x, MAP_X, MAP_RESOLUTION);
    int goal_y_index = poseToIndex(goal_area_y, MAP_Y, MAP_RESOLUTION);  
    for (int y_index=MAP_RESOLUTION-1;y_index>=0;y_index--)
    {
        for(int x_index=0;x_index<MAP_RESOLUTION;x_index++){
            if ((x_index == pos_x_index) &&  (y_index == pos_y_index))
            {
              Serial.print("O");
            }else if ((x_index == goal_x_index) &&  (y_index == goal_y_index))
            {
              Serial.print("X");
            }else {
              Serial.print( (unsigned int)processed_map2[y_index][x_index] );
            }
            Serial.print("  ");
        }
        Serial.println("");
    }
/**/
}
void Mapper::updateMapFeature(byte feature, float y, float x) {
  updateMapFeature( feature, (int)y, (int)x );  
}

void Mapper::updateMapFeature(byte feature, int y, int x)
{
    if (x >= MAP_X || x < 0 || y >= MAP_Y || y < 0)
    {
      Serial.println(F("Error:Invalid co-ordinate"));
      return;
    }

    int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
    int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);  
//    int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;  
    int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;  

    if (eeprom_address > 1023)
    {
        Serial.println(F("Error: EEPROM Address greater than 1023"));
    }
    else
    {
        char value = (char)EEPROM.read(eeprom_address);
        //Only write feature if cell doesn't contain an RFID tag or 
        if (value ==MAP_DEFAULT_FEATURE||value ==MAP_EXPLORED_FEATURE){
          EEPROM.update(eeprom_address, feature);}
    }
    

}


#endif
