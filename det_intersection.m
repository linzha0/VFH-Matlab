function [ distance,theta ] = det_intersection(Barrier, Pos_A,ang_A)

%a function to determine intersection point between two segments
%and return smallest distance value?put the value one by one into 24
%vectors which represtent obstacle around UAV
degree_A=ang_A*180/pi;
if degree_A >=360
    degree_A=degree_A-360;
elseif degree_A <0
    degree_A=degree_A+360;
end

  if degree_A <=90 && degree_A >=0 
      rotate=abs(90-degree_A)/180*pi;
      ca=1;
  
  elseif degree_A <=270 && degree_A>90
      rotate=abs(degree_A-90)/180*pi;
      ca=2;
  
  elseif degree_A >270
      rotate=(360-degree_A+90)/180*pi;
      ca=1;
  end
      
  
for j=0:1:23
    if ca==1 
    theta(j+1)=-rotate-5*pi/36+pi/18*j;
    else
    theta(j+1)=rotate-5*pi/36+pi/18*j;
    end
        
    
    
l=18;%sensor distance
Pos_S=[Pos_A(1)+cos(theta(j+1))*l,Pos_A(2)+sin(theta(j+1))*l];


a=numel(Barrier(:,1));
k=0;

for i=1:1:a % every loop choose 2  adjacent points of barrier
  if i ~= a      
      p1=[Barrier(i,1),Barrier(i,2)];
      p2=[Barrier(i+1,1),Barrier(i+1,2)];
  else
      p1=[Barrier(i,1),Barrier(i,2)];
      p2=[Barrier(1,1),Barrier(1,2)];   
  end
  
  if (p1(1) ~= p2(1)) && (Pos_A(1) ~= Pos_S(1))
   
    k1=(p1(2)-p2(2))/(p1(1)-p2(1));
    b1=p1(2)-p1(1)*k1;
  
    k2=(Pos_S(2)-Pos_A(2))/(Pos_S(1)-Pos_A(1));
    b2=Pos_A(2)-Pos_A(1)*k2;
    
    x=(b2-b1)/(k1-k2);
    y=k1*x+b1;
  
     if k1 == k2
        inter=0;
     else

       [ inter ] = det_range( Pos_A,Pos_S,p1,p2,x,y );
      
     end
  
  elseif (p1(1) == p2(1)) && (Pos_A(1) ~= Pos_S(1))
      k2=(Pos_S(2)-Pos_A(2))/(Pos_S(1)-Pos_A(1));
      b2=Pos_A(2)-Pos_A(1)*k2;
      x=p1(1);
      y=k2*x+b2;
      [ inter ] = det_range( Pos_A,Pos_S,p1,p2,x,y );
      
  elseif (p1(1) ~= p2(1)) && (Pos_A(1) == Pos_S(1))
      k1=(p1(2)-p2(2))/(p1(1)-p2(1));
      b1=p1(2)-p1(1)*k1;
      x=Pos_A(1);
      y=k1*x+b1;
      [ inter ] = det_range( Pos_A,Pos_S,p1,p2,x,y );
  else
      inter=0;
      
  end
  
  if inter == 1 % store values every axis intersection of obstacle
      temp=sqrt((x-Pos_A(1))^2+(y-Pos_A(2))^2);
      k=k+1;
      Temp(1,k)=temp;
  
  
  end
  
  
  
  
end

%determine every sensor area meet obstacle or not, 0 means no.
if k ~=0
    
    dis=min(Temp);
    distance(1,j+1)=dis;
else
    distance(1,j+1)=0;
end
    


end


end

