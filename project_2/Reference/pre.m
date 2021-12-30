function Jthita=pro(T6)
dtor = pi/180;  %degree to rad
rtod = 180/pi;  %rad to degree
d2 = 1;


%inverse kinemati
    
nx=T6(1,1);
ny=T6(2,1);
nz=T6(3,1);

ox=T6(1,2);
oy=T6(2,2);
oz=T6(3,2);

ax=T6(1,3);
ay=T6(2,3);
az=T6(3,3);

px=T6(1,4);
py=T6(2,4);
pz=T6(3,4);
   
    %theta1
    r = sqrt((px)^2+(py)^2);   
    t11 = atan2(py,px)-atan2(d2,sqrt((r)^2-(d2)^2));  
    t12 = atan2(py,px)+atan2(d2,sqrt((r)^2-(d2)^2)); 
    fprintf('t11= %f  \n',t11*rtod);    
       fprintf('t12= %f  \n',t12*rtod);
       
    if abs(t12*rtod)<=120
    t1 = t12+pi;
    dime=2;
    end
    
    if  abs(t11*rtod)<=120
    t1 = t11;  
    dime=1;
    end

   fprintf('t1= %f  \n',t1*rtod);
    s1 = sin(t1);
    c1 = cos(t1);
    %theta2
    t2 = atan2(c1*px+s1*py,pz);
    s2 = sin(t2);
    c2 = cos(t2);
    %distance3
    d3 = s2*(c1*px+s1*py)+c2*pz;
    %theta4
    t44 = atan2(-s1*ax+c1*ay ,c2*(c1*ax+s1*ay)-s2*az);
    while dime == 1
    t4=t44;
    break 
    end
    while dime == 2
    t4=t44-pi;
    break 
    end
    s4 = sin(t4);
    c4 = cos(t4);
    %theta5
    t5 = atan2(c4*(c2*(c1*ax+s1*ay)-s2*az)+s4*(-s1*ax+c1*ay),s2*(c1*ax+s1*ay)+c2*az);    
    s4 = sin(t4);
    c4 = cos(t4);
    %theta6
    s5 = sin(t5);
    c5 = cos(t5);
    coso = c1*ox + s1*oy;
    soco = -s1*ox + c1*oy;
    so = s2*oz;
    co = c2*oz;
    t6 = atan2(-c5*(c4*(c2*coso-so)+s4*soco)+s5*(s2*coso+co), -s4*(c2*coso-so)+c4*soco );
    
    while abs(t11*rtod)>120
    fprintf('Theta1 can not enable\n');
    break 
    end
    while abs(t2*rtod)>75
        fprintf('Theta2 can not enable\n');
    break 
    end
    while abs(t4*rtod)>90
    fprintf('Theta4 can not enable\n');
    break 
    end
    while abs(t5*rtod)>90
    fprintf('Theta5 can not enable\n');
    break 
    end
    while abs(t6*rtod)>170
    fprintf('Theta6 can not enable\n');
    break 
    end
    
    fprintf('(theta1,theta2,distance3,theta4,theta5,theta6)=(%f ,%f ,%f ,%f ,%f ,%f) \n',t1*rtod,t2*rtod,d3,t4*rtod,t5*rtod,t6*rtod);
    
    
    thita=[t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod
       %('Second state:')
       t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod
       %('Third state:')
      t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod
       %('Fourth state:')
      t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod
       %('Fifth state:')
       t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod
       %('Sixth state:')
      t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod
       %('Seventh state:')
       t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod
       %('Eigth state:')
     t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod];

Jthita=thita(1,:)';



    Jthita=[t1*rtod  t2*rtod  d3  t4*rtod  t5*rtod t6*rtod];
    
    
    
              