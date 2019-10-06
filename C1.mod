param Nfe;
param Nv;
var tf >= 1;
param NE = Nfe - 1;
param Nobs;
var hi = tf / NE;
set I := {0..NE};
set I1 := {1..NE};
set Iv := {1..Nv};
param OV{i in I, j in {1..Nobs}, k in {1..4}, n in {1..2}};
param Area{i in {1..Nobs}};
param BV_config{i in {1..(Nv*6)}};
param VP{i in {1..8}};
param AreaVehicle = (VP[1] + VP[2]) * 2 * VP[3];

var x{k in Iv, i in I};
var y{k in Iv, i in I};
var xf{k in Iv, i in I};
var yf{k in Iv, i in I};
var xr{k in Iv, i in I};
var yr{k in Iv, i in I};
var theta{k in Iv, i in I};
var v{k in Iv, i in I};
var a{k in Iv, i in I};
var phy{k in Iv, i in I};
var w{k in Iv, i in I};
var egoV{m in Iv, i in I, j in {1..4}, k in {1..2}};

minimize cost_function:
tf + 0.00001 * sum{k in Iv, i in I}(w[k,i]^2 + a[k,i]^2);

s.t. timer:
tf <= Nfe * 0.4;

### ODEs ###
s.t. DIFF_dxdt {k in Iv, i in I1}:
x[k,i] = x[k,i-1] + hi * v[k,i] * cos(theta[k,i]);
s.t. DIFF_dydt {k in Iv, i in I1}:
y[k,i] = y[k,i-1] + hi * v[k,i] * sin(theta[k,i]);
s.t. DIFF_dvdt {k in Iv, i in I1}:
v[k,i] = v[k,i-1] + hi * a[k,i];
s.t. DIFF_dthetadt {k in Iv, i in I1}:
theta[k,i] = theta[k,i-1] + hi * tan(phy[k,i]) * v[k,i] * VP[4];
s.t. DIFF_dphydt {k in Iv, i in I1}:
phy[k,i] = phy[k,i-1] + hi * w[k,i];

### Equations for vertexes A B C D ###
s.t. RELATIONSHIP_AX {k in Iv, i in I}:
egoV[k,i,1,1] = x[k,i] + VP[1] * cos(theta[k,i]) - VP[3] * sin(theta[k,i]);
s.t. RELATIONSHIP_BX {k in Iv, i in I}:
egoV[k,i,2,1] = x[k,i] + VP[1] * cos(theta[k,i]) + VP[3] * sin(theta[k,i]);
s.t. RELATIONSHIP_CX {k in Iv, i in I}:
egoV[k,i,3,1] = x[k,i] - VP[2] * cos(theta[k,i]) + VP[3] * sin(theta[k,i]);
s.t. RELATIONSHIP_DX {k in Iv, i in I}:
egoV[k,i,4,1] = x[k,i] - VP[2] * cos(theta[k,i]) - VP[3] * sin(theta[k,i]);
s.t. RELATIONSHIP_AY {k in Iv, i in I}:
egoV[k,i,1,2] = y[k,i] + VP[1] * sin(theta[k,i]) + VP[3] * cos(theta[k,i]);
s.t. RELATIONSHIP_BY {k in Iv, i in I}:
egoV[k,i,2,2] = y[k,i] + VP[1] * sin(theta[k,i]) - VP[3] * cos(theta[k,i]);
s.t. RELATIONSHIP_CY {k in Iv, i in I}:
egoV[k,i,3,2] = y[k,i] - VP[2] * sin(theta[k,i]) - VP[3] * cos(theta[k,i]);
s.t. RELATIONSHIP_DY {k in Iv, i in I}:
egoV[k,i,4,2] = y[k,i] - VP[2] * sin(theta[k,i]) + VP[3] * cos(theta[k,i]);

### Equations for xf yf xr yr ###
s.t. RELATIONSHIP_XF {k in Iv, i in I}:
xf[k,i] = x[k,i] + 2.5877 * cos(theta[k,i]);
s.t. RELATIONSHIP_YF {k in Iv, i in I}:
yf[k,i] = y[k,i] + 2.5877 * sin(theta[k,i]);
s.t. RELATIONSHIP_XR {k in Iv, i in I}:
xr[k,i] = x[k,i] + 0.2432 * cos(theta[k,i]);
s.t. RELATIONSHIP_YR {k in Iv, i in I}:
yr[k,i] = y[k,i] + 0.2432 * sin(theta[k,i]);

### Manifold constraints w.r.t. vehicle physics ###
s.t. Bonds_phy {k in Iv, i in I}:
-VP[7] <= phy[k,i] <= VP[7];
s.t. Bonds_a {k in Iv, i in I}:
-VP[6] <= a[k,i] <= VP[6];
s.t. Bonds_v {k in Iv, i in I}:
-VP[5] <= v[k,i] <= VP[5];
s.t. Bonds_w {k in Iv, i in I}:
-VP[8] <= w[k,i] <= VP[8];

### Collision avoidance 1 ###
s.t. ObsVertexOutOfABCD {m in Iv, i in I, j in {1..Nobs}, k in {1..4}}:
0.5 * abs(OV[i,j,k,1] * egoV[m,i,1,2] + egoV[m,i,1,1] * egoV[m,i,2,2] + egoV[m,i,2,1] * OV[i,j,k,2] - OV[i,j,k,1] * egoV[m,i,2,2] - egoV[m,i,1,1] * OV[i,j,k,2] - egoV[m,i,2,1] * egoV[m,i,1,2]) + 
0.5 * abs(OV[i,j,k,1] * egoV[m,i,3,2] + egoV[m,i,3,1] * egoV[m,i,2,2] + egoV[m,i,2,1] * OV[i,j,k,2] - OV[i,j,k,1] * egoV[m,i,2,2] - egoV[m,i,3,1] * OV[i,j,k,2] - egoV[m,i,2,1] * egoV[m,i,3,2]) + 
0.5 * abs(OV[i,j,k,1] * egoV[m,i,3,2] + egoV[m,i,3,1] * egoV[m,i,4,2] + egoV[m,i,4,1] * OV[i,j,k,2] - OV[i,j,k,1] * egoV[m,i,4,2] - egoV[m,i,3,1] * OV[i,j,k,2] - egoV[m,i,4,1] * egoV[m,i,3,2]) + 
0.5 * abs(OV[i,j,k,1] * egoV[m,i,1,2] + egoV[m,i,1,1] * egoV[m,i,4,2] + egoV[m,i,4,1] * OV[i,j,k,2] - OV[i,j,k,1] * egoV[m,i,4,2] - egoV[m,i,1,1] * OV[i,j,k,2] - egoV[m,i,4,1] * egoV[m,i,1,2]) >= AreaVehicle + 0.2;

s.t. CarVertexOutOfObstacle {m in Iv, i in I, j in {1..Nobs}, k in {1..4}}:
0.5 * abs(egoV[m,i,k,1] * OV[i,j,1,2] + OV[i,j,1,1] * OV[i,j,2,2] + OV[i,j,2,1] * egoV[m,i,k,2] - egoV[m,i,k,1] * OV[i,j,2,2] - OV[i,j,1,1] * egoV[m,i,k,2] - OV[i,j,2,1] * OV[i,j,1,2]) + 
0.5 * abs(egoV[m,i,k,1] * OV[i,j,3,2] + OV[i,j,3,1] * OV[i,j,2,2] + OV[i,j,2,1] * egoV[m,i,k,2] - egoV[m,i,k,1] * OV[i,j,2,2] - OV[i,j,3,1] * egoV[m,i,k,2] - OV[i,j,2,1] * OV[i,j,3,2]) + 
0.5 * abs(egoV[m,i,k,1] * OV[i,j,3,2] + OV[i,j,3,1] * OV[i,j,4,2] + OV[i,j,4,1] * egoV[m,i,k,2] - egoV[m,i,k,1] * OV[i,j,4,2] - OV[i,j,3,1] * egoV[m,i,k,2] - OV[i,j,4,1] * OV[i,j,3,2]) + 
0.5 * abs(egoV[m,i,k,1] * OV[i,j,1,2] + OV[i,j,1,1] * OV[i,j,4,2] + OV[i,j,4,1] * egoV[m,i,k,2] - egoV[m,i,k,1] * OV[i,j,4,2] - OV[i,j,1,1] * egoV[m,i,k,2] - OV[i,j,4,1] * OV[i,j,1,2]) >= Area[j] + 0.2;

### Collision avoidance 2 ###
s.t. CarVertexInBox {m in Iv, i in I, j in {1..4}, k in {1..2}}:
-20 <= egoV[m, i,j,k] <= 20;

### Collision avoidance 3 ###
s.t. VehicleItoJff {i in {1..(Nv-1)}, j in {(i+1)..Nv}, kk in I}:
(xf[i,kk] - xf[j,kk])^2 + (yf[i,kk] - yf[j,kk])^2 >= 9.2680;
s.t. VehicleItoJrr {i in {1..(Nv-1)}, j in {(i+1)..Nv}, kk in I}:
(xr[i,kk] - xr[j,kk])^2 + (yr[i,kk] - yr[j,kk])^2 >= 9.2680;
s.t. VehicleItoJfr {i in {1..(Nv-1)}, j in {(i+1)..Nv}, kk in I}:
(xf[i,kk] - xr[j,kk])^2 + (yf[i,kk] - yr[j,kk])^2 >= 9.2680;
s.t. VehicleItoJrf {i in {1..(Nv-1)}, j in {(i+1)..Nv}, kk in I}:
(xr[i,kk] - xf[j,kk])^2 + (yr[i,kk] - yf[j,kk])^2 >= 9.2680;

############# Two-Point Boundary Values #############
s.t. EQ_init_x {m in Iv}:
x[m,0] = BV_config[6*(m-1)+1];
s.t. EQ_init_y {m in Iv}:
y[m,0] = BV_config[6*(m-1)+2];
s.t. EQ_init_theta {m in Iv}:
theta[m,0] = BV_config[6*(m-1)+3];
s.t. EQ_init_v {m in Iv}:
v[m,0] = 0;
s.t. EQ_init_a {m in Iv}:
a[m,0] = 0;
s.t. EQ_init_phy {m in Iv}:
phy[m,0] = 0;
s.t. EQ_init_w {m in Iv}:
w[m,0] = 0;

s.t. EQ_end_x {m in Iv}:
x[m,NE] = BV_config[6*(m-1)+4];
s.t. EQ_end_y {m in Iv}:
y[m,NE] = BV_config[6*(m-1)+5];
s.t. EQ_end_theta1 {m in Iv}:
sin(theta[m,NE]) = sin(BV_config[6*(m-1)+6]);
s.t. EQ_end_theta2 {m in Iv}:
cos(theta[m,NE]) = cos(BV_config[6*(m-1)+6]);
s.t. EQ_end_v {m in Iv}:
v[m,NE] = 0;
s.t. EQ_end_a {m in Iv}:
a[m,NE] = 0;
s.t. EQ_end_phy {m in Iv}:
phy[m,NE] = 0;
s.t. EQ_end_w {m in Iv}:
w[m,NE] = 0;

data;
param Nfe := include Nfe;
param Nv := include Nv;
param Nobs := include Nobs;
param OV := include OV;
param Area := include Area;
param BV_config := include BV_config;
param VP := include VP;