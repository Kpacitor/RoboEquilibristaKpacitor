
double kp = 22.0;
double kd = 0.9; 
double ki = 0.9;

int OUTMAX = 200;
int OUTMIN = -200;

double SetPoint = -4.5;

float lastInput = 0.0;

double ITerm = 0.0;

double Compute(double input){

  /*Calculo do Erro*/
  double erro = SetPoint - input;

  ITerm+= (ki * erro);

  if(ITerm > OUTMAX){
    ITerm = OUTMAX;
  }else if(ITerm <= OUTMIN){
    ITerm = OUTMIN;
  }


  /* Diferença da entrada atual - anterior*/
  double dInput = input - lastInput;

  /* Calculo do PID */
  double output = kp * erro + ITerm + kd * dInput;


  if(output > OUTMAX){
    output = OUTMAX;
  }else if(output <= OUTMIN){
    output = OUTMIN;
  }

  lastInput = input;

  return output;
  
}



