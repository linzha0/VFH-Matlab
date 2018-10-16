function [ inter ] = det_range( Pos_A,Pos_S,p1,p2,x,y )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
P1=[p1(1),p2(1)];
      X1=[min(P1),max(P1)];
      P2=[p1(2),p2(2)];
      Y1=[min(P2),max(P2)];
      
      if (x >= X1(1) && x <= X1(2)) && (y >= Y1(1) && y<= Y1(2))
          in1 =1;
      else 
          in1 =0;
      end
      
      P3=[Pos_A(1),Pos_S(1)];
      X2=[min(P3),max(P3)];
      P4=[Pos_A(2),Pos_S(2)];
      Y2=[min(P4),max(P4)];
      
      if (x >= X2(1) && x <= X2(2)) && (y >= Y2(1) && y<= Y2(2))
          in2=1;
      else
          in2=0;
      end
      
      if in1 ==1 && in2 ==1
          inter=1;
      else
          inter=0;
      end

end

