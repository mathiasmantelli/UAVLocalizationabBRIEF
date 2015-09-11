#include "readerlog.h"


ReaderLog::ReaderLog(string endereco)
{ //
    file_input.open(endereco.c_str());
    if(!file_input.is_open()){
        cout << "File: " << endereco << " does not exist" << endl;
        exit(0);
    }
    imu_informations.clear();
    imu2_informations.clear();
    gps_informations.clear();
}

void ReaderLog::Read(){
    if(file_input.is_open()){
        while(getline(file_input,input_line)){
            //SELECTING THE IMU INFORMATIONS
            if(input_line[0] == 'I' && input_line[1] == 'M' && input_line[2] == 'U' && input_line[3] != '2'){
                sensor_IMU(input_line);
            }
            else if(input_line[0] == 'I' && input_line[1] == 'M' && input_line[2] == 'U' && input_line[3] == '2'){
                sensor_IMU2(input_line);
            }
            else if(input_line[0] == 'G' && input_line[1] == 'P' && input_line[2] == 'S'){
                sensor_GPS(input_line);
            }
            else if(input_line[0] == 'A' && input_line[1] == 'T' && input_line[2] == 'T'){
                sensor_ATT(input_line);
            }
        }
//        cout<<"cabou"<<endl;
    }
    file_input.close();

}

void ReaderLog::sensor_IMU(string input_line){
    double aux;
    int count = 0;
    IMU temp;
    for(unsigned int i = 4; i < input_line.size();i++){
        //SEPARATING THE NUMBERS
        if(isalpha(input_line[i]) || isdigit(input_line[i]) || input_line[i] == '.' || input_line[i] == '-' || input_line[i] == ' '){
            buffer += input_line[i];
        }else{
            //FIND THE "," OR END LINE
            if(buffer.size() > 1){
                stringstream in(buffer);
                in>>aux;
                temp.values[count++] = aux;
                buffer.clear();
            }
        }
    }
    imu_informations.push_back(temp);
}

void ReaderLog::sensor_IMU2(string input_line){
    double aux;
    int count = 0;
    IMU temp;
    for(unsigned int i = 5; i < input_line.size();i++){
        //SEPARATING THE NUMBERS
        if(isalpha(input_line[i]) || isdigit(input_line[i]) || input_line[i] == '.' || input_line[i] == '-' || input_line[i] == ' '){
            buffer += input_line[i];
        }else{
            //FIND THE "," OR END LINE
            if(buffer.size() > 1){
                stringstream in(buffer);
                in>>aux;
                temp.values[count++] = aux;
                buffer.clear();
            }
        }
    }
    imu2_informations.push_back(temp);
}

void ReaderLog::sensor_GPS(string input_line){
    double aux;
    int count = 0;
    GPS temp;
    for(unsigned int i = 4; i < input_line.size();i++){
        //SEPARATING THE NUMBERS
        if(isalpha(input_line[i]) || isdigit(input_line[i]) || input_line[i] == '.' || input_line[i] == '-' || input_line[i] == ' '){
            buffer += input_line[i];
        }else{
            //FIND THE "," OR END LINE
            if(buffer[0] != ','){
                stringstream in(buffer);
                in>>aux;
                temp.values[count++] = aux;
                buffer.clear();
            }
        }
    }
    gps_informations.push_back(temp);
}

void ReaderLog::sensor_ATT(string input_line){
    double aux;
    int count = 0;
    ATT temp;
    for(unsigned int i = 4; i < input_line.size();i++){
        //SEPARATING THE NUMBERS
        if(isalpha(input_line[i]) || isdigit(input_line[i]) || input_line[i] == '.' || input_line[i] == '-' || input_line[i] == ' '){
            buffer += input_line[i];
        }else{
            //FIND THE "," OR END LINE
            if(buffer[0] != ','){
                stringstream in(buffer);
                in>>aux;
                temp.values[count++] = aux;
                buffer.clear();
            }
        }
    }
    att_informations.push_back(temp);
}

vector<IMU> ReaderLog::get_sensor_IMU(){
    return imu_informations;
}

vector<IMU> ReaderLog::get_sensor_IMU2(){
    return imu2_informations;
}

vector<GPS> ReaderLog::get_sensor_GPS(){
    return gps_informations;
}

vector<ATT> ReaderLog::get_sensor_ATT(){
    return att_informations;
}
