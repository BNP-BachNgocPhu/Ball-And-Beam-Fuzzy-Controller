# **Ball-And-Beam-Fuzzy-Controller**
# **1. Project introduction**
## Decription
The "Ball and Beam" project uses a Fuzzy PD controller to keep the ball stable on the beam by adjusting the beam's tilt angle.  
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/bde807c6-3d84-441f-8be8-4c07fcfa476b" alt="Project Overview">
  <br>
  <em>Figure 1: Ball and beam system </em>
</p>

## Objective
- Control the beam tilt angle to keep the ball in the desired position.
- Applying the fuzzy control algorithm (Fuzzy Logic) combined with the PD controller.

## Components
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/0405d217-b333-4623-8c0b-384f909e4893" alt="Project Overview">
  <br>
  <em>Figure 2: All Components in this system </em>
</p>

- **STM32F103C8T6 Microcontroller**: Data processing and servo control.
- **HC-SR04 Ultrasonic Sensor**: Measures the distance of the ball.
- **MG996R Servo**: Adjusts the tilt angle of the beam.
- **3x 3.7V battery**: Provides power to the system.
- **Voltage Reduction Circuit**: Stabilizes the power supply.

# **2. Wiring diagram**
The system is connected according to the following diagram:
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/e4a26b04-86f0-4d9f-922d-32c213e581f8" alt="Project Overview">
  <br>
  <em>Figure 3: Wiring diagram </em>
</p>

```plaintext
HC-SR04 (Measure the ball position) -> STM32F103C8T6
VCC   -> 5V
Trig  -> PB12
Echo  -> PB13
GND   -> GND

HC-SR04 (Measure setpoint position) -> STM32F103C8T6
VCC   -> 5V
Trig  -> PA4
Echo  -> PA5
GND   -> GND

Servo MG996R (Beam tilt control) -> STM32F103C8T6
VCC     -> 5V
GND     -> GND
Signal  -> PA0

LCD I2C (Display ball position and setpoint information) -> STM32F103C8T6
VCC   -> 5V
GND   -> GND
SDA   -> PB7
SCL   -> PB6

3x Pin 3.7V (11.1V) power -> Voltage Reduction Circuit -> STM32F103C8T6
Vin   -> 11.1V
Vout  -> 5V
GND   -> GND
```

# **3. Fuzzy PD controller**
## Introduction to Fuzzy PD Controller
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/62ead332-9700-4a03-822c-9c7a4976e1c0" alt="Project Overview">
  <br>
  <em>Figure 4: The system block diagram on Matlab </em>
</p>

Fuzzy PD controller is used to control Ball and Beam system. In which:
- **Proportional-Derivative PD controller** helps to react quickly to errors and the rate of change of errors.
- **Fuzzy Logic** helps to process non-linear data and optimize control performance.

## Design steps
### 1. **Determine the input and output variables** 
Input/Output variables of the fuzzy PD controller:  
- Two input variables: Error and error rate of change (e, edot)  
- Output variable: Servo rotation angle (theta)   

Normalization of Input/Output Variables of the Controller
- Error: e(t) = xd(t) - x(t), -0.5 <= e(t) <= 0.5 (m) 		    => K1 = 1/0.5 
- Error rate of change: -0.9<= edot(t) <= 0.9 (m/s) 		    => K2 = 1/0.9 
- Output variable (degree): -pi/3<= theta < pi/3 (rad) 			=> K3 = pi/3
```cpp
float Ke =1/50.000;
float Kedot = 1/90.000;
float Ku = 70; #degree
```
### 2. **Defining language values for Input/Output variables**
**Error variable e**
```cpp
// Error
e[0] = hlt_hinhthang(error, -3, -2 , ce[0], ce[1]); // Negative Big (NB)
e[1] = hlt_hinhthang(error, ce[0], ce[1] , ce[1], ce[2]); // Negative Small (NS)
e[2] = hlt_hinhthang(error, ce[1], ce[2] , ce[2], ce[3]); // Zero (ZE)
e[3] = hlt_hinhthang(error, ce[2], ce[3] , ce[3], ce[4]); // Positive Small (PS)
e[4] = hlt_hinhthang(error, ce[3], ce[4] , 2 , 3); // Positive Big (PB)

float ce[5] = {-0.75, -0.25, 0, 0.25, 0.75}; // X-axis values of error variable e
```
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/0773c6ce-ba56-40a9-9a04-299575627c1d" alt="Project Overview">
  <br>
  <em>Figure 5: Defining language values for error variable e </em>
</p>

**Error rate variable ė**
```cpp
// Error rate
edot[0] = hlt_hinhthang(errordot, -3, -2, cedot[0], cedot[1]); // Negative Big (NB)
edot[1] = hlt_hinhthang(errordot, cedot[0], cedot[1], cedot[1], cedot[2]); // Negative Small (NS)
edot[2] = hlt_hinhthang(errordot, cedot[1], cedot[2], cedot[2], cedot[3]); // Zero (ZE)
edot[3] = hlt_hinhthang(errordot, cedot[2], cedot[3], cedot[3], cedot[4]); // Positive Small (PS)
edot[4] = hlt_hinhthang(errordot, cedot[3], cedot[4], 2, 3); // Positive Big (PB)

float cedot[5] = {-0.75, -0.42, 0, 0.42, 0.75}; // X-axis values of error rate variable edot
```
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/74fe602e-7693-4ad1-9401-d04d427f4e48" alt="Project Overview">
  <br>
  <em>Figure 6: Defining language values for error rate variable ė </em>
</p>

**Output variable θ (Servo Angle)*
```cpp
float y_out[7] = {-1, -0.8, -0.4, 0, 0.4, 0.8, 1}; // X-axis values of the linguistic servo angle
```
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/df22f013-9348-4b5c-822f-a5c6566195e0" alt="Project Overview">
  <br>
  <em>Figure 7: Defining language values for output variable θ </em>
</p>

### 3. **Some Fuzzy rules based on experience**
I will show 5 rules based on experience. The remaining 20 rules can be identified similarly, some of them contradicting each other.
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/5cfd1e33-b5b2-41f1-bcb7-381faa3223fd" alt="Project Overview">
  <br>
  <em>Figure 8: Rule 1 is typical which continues to maintain the same state </em>
</p><br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/b4a8a6ba-bfc9-434c-a627-02be8bc2c3c9" alt="Project Overview">
  <br>
  <em>Figure 9: Rule 2 shows 1 case which the beam needs to be raised a little </em>
</p><br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/255618eb-abd1-4f45-90d7-5bb22558a9e2" alt="Project Overview">
  <br>
  <em>Figure 10: Rule 3 shows 1 another case which the beam needs to be raised a little </em>
</p><br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/6c94385d-949f-4e37-ad8c-32d51ddd522d" alt="Project Overview">
  <br>
  <em>Figure 11: Rule 4 shows 1 case which the beam needs to be raised a medium high </em>
</p><br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/86ab6306-6a24-4ef4-89e6-cf349cb972ca" alt="Project Overview">
  <br>
  <em>Figure 12: Rule 5 shows 1 case another which the beam continues to maintain that state </em>
</p>

**FUZZY RULE TABLE**
<div align="center">

| Theta  | **NB**  | **NS**  | **ZE**  | **PS**  | **PB**  |
|--------|-----|-----|-----|-----|-----|
| **NB** | NB  | NB  | NM  | NS  | ZE  |
| **NS** | NB  | <span style="color:red">NM</span>  | <span style="color:red">NS</span>  | <span style="color:red">ZE</span>  | PS  |
| **ZE** | <span style="color:red">NM</span>  | <span style="color:red">NS</span>  |  <span style="color:red">ZE</span> | PS  | PM  |
| **PS** | NS  | ZE  | PS  | PM  | PB  |
| **PB** | ZE  | PS  | PM  | PB  | PB  |

</div>

```cpp
    // Calculate y_tam
    if(((i==0)&&(j==0))||((i==0)&&(j==1))||((i==1)&&(j==0)))
    {
        y_tam[i][j] = y_out[6]; // Position PB
    }
    else if(((i==2)&&(j==0))||((i==1)&&(j==1))||((i==0)&&(j==2)))
    {
        y_tam[i][j] = y_out[5]; // Position PM
    }
    else if(((i==3)&&(j==0))||((i==2)&&(j==1))||((i==1)&&(j==2))||((i==0)&&(j==3)))
    {
        y_tam[i][j] = y_out[4]; // Position PS
    }
    else if(((i==4)&&(j==0))||((i==3)&&(j==1))||((i==2)&&(j==2))||((i==1)&&(j==3))||((i==0)&&(j==4)))
    {
        y_tam[i][j] = y_out[3]; // Position ZE
    }
    else if(((i==4)&&(j==1))||((i==3)&&(j==2))||((i==2)&&(j==3))||((i==1)&&(j==4)))
    {
        y_tam[i][j] = y_out[2]; // Position NS
    }
    else if(((i==4)&&(j==2))||((i==3)&&(j==3))||((i==2)&&(j==4)))
    {
        y_tam[i][j] = y_out[1]; // Position NM
    }
    else
    {
        y_tam[i][j] = y_out[0]; // Position NB
    }
```

### **4. MAX-PROD Method**
The MAX-PROD method is an inference technique in fuzzy systems that utilizes:  
- Product operation: To calculate the activation level of each rule.  
- MAX function: To select the maximum value among the outputs of the rules. 

```cpp
beta[i][j] = e[i]*edot[j];
```
### **5. Weighted Average Defuzzification Method**
The Weighted Average Method is a common defuzzification technique that calculates the output by taking the weighted average of the result values from the fuzzy set.
```cpp
  float tuso = 0.0;
  float mauso = 0.0;

  for (int i =0; i<5; i++)
  {
    for(int j=0; j<5; j++)
    {
      tuso = tuso + (beta[i][j])*(y_tam[i][j]);
      mauso = mauso + beta[i][j];
    }
  }
  y_sao = tuso/mauso;
```
# **4. Simulation**
These set values ​​correspond to the set values ​​when running the experiment.
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/ed114ddf-0081-47a1-9a68-76b82decd525" alt="Project Overview">
  <br>
  <em>Figure 13: Set values to simulation </em>
</p>
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/f23288a4-017d-491c-ba75-391dce7e34b0" alt="Project Overview">
  <br>
  <em>Figure 14: Simulate result </em>
</p>

**Comment on simulation results**  
- The control results show that the system tracks quite well under randomly changing set signals.

# **5. Some model images**
<br>
<p align="center">
  <img src="https://github.com/user-attachments/assets/d7d7739b-651a-4647-b0bd-7f9d41c1df06" alt="Project Overview">
  <br>
  <img src="https://github.com/user-attachments/assets/d9d8c35f-9918-4bbe-87eb-c7f18d2cafb1" alt="Project Overview">
  <br>
  <img src="https://github.com/user-attachments/assets/0f8ba885-9c90-4a35-a524-8bf678858edf" alt="Project Overview">
  <br>
  <img src="https://github.com/user-attachments/assets/5ab5d602-91a6-4c5a-a5d1-29ada6eae0ba" alt="Project Overview">
  <br>
  <em>Figure 15: Some model images </em>
</p>

# **6. Experimental**
### 🔗 Ball and Beam Using Fuzzy PD Controller Experimental
<div align="center">

[![Button Operation](https://github.com/user-attachments/assets/48bdf335-79be-4da9-859f-3cf27b715825)](https://youtu.be/Os6NDnMXoM8)
</div>

**Experimental results**

<div align="center">

| setpoint | 17     | 8      | 27     | 16     | 30     | 17     | 8      | 32     | 19     |
|----------|--------|--------|--------|--------|--------|--------|--------|--------|--------|
| d_out    | 17.349 | 8.3639 | 27.839 | 16.099 | 30.289 | 16.709 | 8.4679 | 32.029 | 18.999 |

</div>


**Comments on experimental results**  
The control results show that the system follows the randomly changing signals quite well, however, 
there is overshoot and the steady-state error still exists.  
**Some comments on experimental results compared with simulation**  
The experimental results are different from the simulation results due to the following reasons:
- Ignoring friction force
- Difference between simulated and experimental “Ball” objects
- Simulation only considers the mass of “Ball” and ignores the mass of “Beam”
- Inertial force of “Ball”
- Error due to measuring device (ultrasonic sensor)
- Some parameters in the simulation are estimates
...