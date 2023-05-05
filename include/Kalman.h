#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>


//KALMAN Filter stuff
#define KALMAN_CHECK true
#define KALMAN_VERBOSE false

using namespace BLA;

//Kalman filter constants
const int N_state = 3; //state vector for pos, vel, accel
const int N_obs = 2; //observing pos, accel

const float n_p = .17;
const float n_a = .12;

const float m1 = 0.1;
const float m2 = 0.1;
const float m3 = 0.8;


template<int dim, class ElemT> struct Symmetric{
  mutable ElemT m[dim*(dim+1)/2];
  typedef ElemT elem_t;
  ElemT &operator()(int row, int col) const{
    static ElemT dummy;
    if(col < dim && row < dim){
      if(col < row){ // swap row and col
        ElemT temp = row;
        row = col;
        col = temp;
      }
      return m[(2*dim-row+1)*row/2+col-row];
    }else
      return (dummy = 0);
  }
};

template<int Nstate, int Nobs, int Ncom=0, class MemF = Array<Nstate, Nstate, float>>
class Kalman
{
  private:
  void _update(BLA::Matrix<Nobs>& obs, BLA::Matrix<Nstate> comstate);
  BLA::Identity<Nstate, Nstate> Id;

  public:
  BLA::Matrix<Nstate, Nstate, MemF> F;
  BLA::Matrix<Nobs, Nstate> H;
  BLA::Matrix<Nstate, Ncom> B;
  BLA::Matrix<Nstate, Nstate, Symmetric<Nstate,float>> Q;
  BLA::Matrix<Nobs, Nobs, Symmetric<Nobs, float>> R;

  BLA::Matrix<Nstate, Nstate, Symmetric<Nstate,float>> P;
  BLA::Matrix<Nstate> x;

  int k_status;

  void update(BLA::Matrix<Nobs>& obs);

  void update(BLA::Matrix<Nobs> obs, BLA::Matrix<Ncom> com);

  Kalman<Nstate, Nobs, Ncom, MemF>();

  BLA::Matrix<Nstate> getState();
  
};

template <int Nstate, int Nobs, int Ncom, class MemF>
void Kalman<Nstate, Nobs, Ncom, MemF>::_update(BLA::Matrix<Nobs>& obs, BLA::Matrix<Nstate> comstate)
{
  if (KALMAN_CHECK)
  {
    for (int i=0; i<Nobs; i++)
    {
      if (isnan(obs(i)) || isinf(obs(i)))
      {
        if (KALMAN_VERBOSE)
        { Serial.println("KALMAN:ERROR: observation nan or inf");
        
        }
        k_status = 1;
        return;
      }
    }
  }
  BLA::Matrix<Nobs, Nobs> S;
  BLA::Matrix<Nstate, Nobs> K;

  this->x = this->F * this->x + comstate;
  this->P = this->F * this->P * (~this->F) + this->Q;

  S = this->H * this->P * (~this->H) + this->R;
  bool is_nonsingular = Invert(S);
  K = P*(~H)*S;

  if (is_nonsingular)
  {
    this->x += K * (obs - this->H * this->x);
    this->P = (this->Id - K * this->H) * this->P;

    if (KALMAN_CHECK)
  {
    for (int i=0; i<Nstate; i++)
    {
      if (isnan(x(i)) || isinf(x(i)))
      {
        if (KALMAN_VERBOSE)
        { Serial.println("KALMAN:ERROR: estimated vector nan or inf");
        
        }
        k_status = 1;
        return;
      }
    }
  }
    
  }
  else
  {
    if (KALMAN_VERBOSE) Serial.println(F("KALMAN:ERROR: could not invert S matrix, reset p matrix"));
    this->P.Fill(0.0);
  }
  
}

template< int Nstate, int Nobs, int Ncom, class MemF>
void Kalman<Nstate,Nobs,Ncom,MemF>::update(BLA::Matrix<Nobs>& obs)
{
  BLA::Zeros<Nstate> NULLCOMSTATE;
  _update(obs,NULLCOMSTATE);
}

template< int Nstate, int Nobs, int Ncom, class MemF>
Kalman<Nstate,Nobs,Ncom,MemF>::Kalman()
{
  if (KALMAN_VERBOSE)
  {
    if ((Nstate<=1) || (Nobs<=1)) Serial.println(F("Kalman:Error Nstate and Nobs must be >1"));
  }
  this->P.Fill(0.0);
  this->x.Fill(0.0);
}

template< int Nstate, int Nobs, int Ncom, class MemF>
BLA::Matrix<Nstate> Kalman<Nstate,Nobs,Ncom,MemF>::getState()
{
  BLA::Matrix<Nstate> out;
  for (int i=0; i<Nstate; i++) out(i) = this->x(i);
  return out;
}