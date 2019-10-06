param Nfe;
param Nv;
param tf;
param NE = Nfe - 1;
param Nobs;
param hi = tf / NE;
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
1;

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

data;
param Nfe := include Nfe;
param Nv := include Nv;
param tf := include TF;
param Nobs := include Nobs;
param OV := include OV;
param Area := include Area;
param BV_config := include BV_config;
param VP := include VP;