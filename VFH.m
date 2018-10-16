clc;clear;close all
format compact
global mapObj_gobal Pos_A
Pos_A=[10,5];
Goal_A=[26,30];
v=0.8;
r_A=2.5;
t=1;
mapObj_gobal = containers.Map('KeyType','char','ValueType','int32'); 
aaa=0;

p=1;
position(p,1)=Pos_A(1);
position(p,2)=Pos_A(2);

% update a new position if A

Barrier1=[2 , 10;2 , 20; 14, 20;14, 18;4,  18;4,  10];
Barrier2=[24, 10;26, 10;26, 25;24, 25];
Barrier3=[11,  30;15,  30;15, 34;11, 34];

[ getgoal ] = Getgoal( Pos_A,Goal_A );
while (getgoal ==0)
 
 [ ang_A, v_Abest] = FindBestVel( Pos_A, Goal_A, v );   
 Barrier=Barrier1;
 [ distance,theta ] = det_intersection(Barrier, Pos_A,ang_A);
 Dis1=distance;      
 Barrier=Barrier2;
[ distance,theta ] = det_intersection(Barrier, Pos_A,ang_A);
 Dis2=distance;     
 Barrier=Barrier3;
[ distance,theta ] = det_intersection(Barrier, Pos_A,ang_A);
 Dis3=distance;
 Dis=Dis1+Dis2+Dis3;%return sensor detection

   %translate cartesian into histogram grid   

   %return which grid near obstacle in range of sensor

   
    for j=0:1:23
        
       %grid is coordinate from sensor farme
        Grid(j+1,1)=cos(theta(j+1))*Dis(j+1);
        Grid(j+1,2)=sin(theta(j+1))*Dis(j+1);
        
        if (Grid(j+1,1) == 0) && (Grid(j+1,2) == 0)
            
            continue
        else
           B=[Grid(j+1,1)+Pos_A(1),Grid(j+1,2)+Pos_A(2)];
           for i=1:1:2
              if B(i) >= 0
                  coor(i)=ceil(B(i));
              else
                  coor(i)=floor(B(i));
              end  
           end
           %store coordinates name
           coor_char1=int2str(coor(1)); 
           coor_char2=int2str(coor(2));
           Char=strcat(coor_char1,'a',coor_char2);
           key={Char};
           %store date corresponding key into date structure
           
           tf=isKey(mapObj_gobal,key);
           if tf ==0
               value=1;
               newmap=containers.Map(key,value);
               mapObj_gobal=[mapObj_gobal; newmap];
           else
               key_cell=values(mapObj_gobal,key);
               key_num=cell2mat(key_cell);
               value=key_num+1;
               if value >5
                   value =5;
               end
               newmap=containers.Map(key,value);
               mapObj_gobal=[mapObj_gobal; newmap];
           end
        end 
    end
    
    
    % Grid_A is the gobal grid coordinate of agent
   A=[Pos_A(1),Pos_A(2)];
   for i=1:1:2
       if A(i) >= 0
           Grid_A(i)=ceil(A(i));
       else
           Grid_A(i)=floor(A(i));
       end
   end
      
    %search cell encolse active window
window=zeros(15,15);
angle=zeros(15,15);
k=zeros(15,15);
d=zeros(15,15);

Leng = length(mapObj_gobal);
Key_gobal= keys(mapObj_gobal);
for i=1:1:Leng
    key_char=char(Key_gobal(i));
    det = isstrprop(key_char,'alpha');
    [col]=find(det>0,1);
    x=str2double(key_char(1:col-1));
    y=str2double(key_char(col+1:end));
    if (x >=Grid_A(1)-7 && x<=Grid_A(1)+7) && (y >=Grid_A(2)-7 && y<=Grid_A(2)+7)
        active_val=values(mapObj_gobal,Key_gobal(i));
        a=abs(Grid_A(2)+7 -y)+1;
        b=abs(Grid_A(1)-7 -x)+1;
        window(a,b)=cell2mat(active_val);
        angle(a,b)=atan2(y-Pos_A(2),x-Pos_A(1))*180/pi;
        if angle(a,b) < 0
            angle(a,b)= angle(a,b)+360;
        end
        if angle(a,b) ==0
            k(a,b)=1;
        else
            k(a,b)=ceil(angle(a,b)/5);
        end
       d(a,b)=sqrt( (x-Pos_A(1))^2+(y-Pos_A(2))^2);
    end
    
end
  
H=zeros(1,72);
for i=1:1:15
    for j=1:1:15
        if window(i,j) > 0
           m=window(i,j)^2*(10.6-d(i,j));
           section=k(i,j);
           H(section)=H(section)+m;
        end
    end
end

for i=1:1:72
    index=[i-4,i-3,i-2,i-1,i,i+1,i+2,i+3,i+4];
    for j=1:1:9
        if index(j)>72
            index(j)=index(j)-72;
        elseif index(j) <1
            index(j)=index(j)+72;
        end
    end
    H_s(i)=([1,2,3,4,5,4,3,2,1]*[H(index(1));H(index(2));H(index(3));H(index(4));H(index(5));H(index(6));H(index(7));H(index(8));H(index(9))])/11;
end


%%% make deiretion with polar digram
H_threshold=find(H_s <10);
L=length(H_threshold);
Set=0;
count=1;
i=0;
box=[];
while 1
    i=i+1;
    if i == L
           Set=Set+1;
           boundry2=H_threshold(i);
           box(Set,2)=boundry2;
           box(Set,1)=count;
           count=1;
       
    else
            
           if H_threshold(i+1)-H_threshold(i) ==1
             count=count+1;
             continue
    
           end
             Set=Set+1;
             boundry2=H_threshold(i);
             box(Set,2)=boundry2;
             box(Set,1)=count;
             count=1;
    end
    if i ==L
        break
    end
        
end
        
if H_threshold(end) ==72 && H_threshold(1)==1
    box(1,1)=box(1,1)+box(end,1);
    box(end,:)=[];
end



%determine the next step veclocity angle  
 ang_best=ang_A;
 
 deter_box=isempty(box); 
if deter_box==1
    V=v_Abest;
else
     Ang_future=[];
     Ang_point=[];
     Boundry=[];
    [l_box,qwe]=size(box);
    for boxnum=1:1:l_box
        if box(boxnum,1) >=50
                   if box(boxnum,2)-box(boxnum,1) >=0
                      Boundry(boxnum,1)=box(boxnum,2)-box(boxnum,1)+1;
                      Boundry(boxnum,2)=box(boxnum,2);
                   else
                      Boundry(boxnum,1)=box(boxnum,2)-box(boxnum,1)+73;
                      Boundry(boxnum,2)=box(boxnum,2);
                   end
                   [ang_future ] =det_future_ang( Boundry(boxnum,:),ang_A);
                  
         else %box(boxnum,1)>12 && box(boxnum,1) <25
                   if box(boxnum,2)-box(boxnum,1) >=0
                      Boundry(boxnum,1)=box(boxnum,2)-box(boxnum,1)+1;
                      Boundry(boxnum,2)=box(boxnum,2);
                      
                      %middle=((2.5+5*(Boundry(boxnum,1)-1))+(2.5+5*(Boundry(boxnum,2)-1)))/2;
                      middle=(5*(Boundry(boxnum,2))-5*(Boundry(boxnum,1)-1))/2;
                      ang_future=5*(Boundry(boxnum,2))-middle;
                      if  ang_future<0
                          ang_future=360+ang_future;
                      end
                      
                   else
                      Boundry(boxnum,1)=box(boxnum,2)-box(boxnum,1)+73;
                      Boundry(boxnum,2)=box(boxnum,2);
                      
                      middle=(360-(5*(Boundry(boxnum,1)-1))+5*(Boundry(boxnum,2)))/2;
                      ang_future=5*(Boundry(boxnum,2))-middle;
                      if  ang_future<0
                          ang_future=360+ang_future;
                      end
                   end
%         else
%             continue
        end
        ang_future=ang_future*pi/180;
        Ang_future(boxnum)=ang_future;
        Ang_x=cos(ang_future)-cos(ang_A);
        Ang_y=sin(ang_future)-sin(ang_A);
        Ang_point(boxnum)=sqrt(Ang_x^2+Ang_y^2);
    end
    
[M,IN]=min(Ang_point);
ang_A=Ang_future(IN); 
            
V=[v*cos(ang_A),v*sin(ang_A)];
            

end

simX1=Pos_A(1)+V(1)*t;
simY1=Pos_A(2)+V(2)*t;
Pos_A=[simX1,simY1];



p=p+1;

position(p,1)=Pos_A(1);
position(p,2)=Pos_A(2);

[ getgoal ] = Getgoal( Pos_A,Goal_A );
% 
if p==50
    break
end
% % if aaa==1
% %     break
% % end
% %if Pos_A(2)>25.0
   % %  break
 % %end
end



h=figure('numbertitle','off','name','FVH');
plot([0,33],[0,49],'.');
plot(Goal_A(1),Goal_A(2),'*');
hold on

fill(Barrier1(:,1),Barrier1(:,2),[0,0,0]);
fill(Barrier2(:,1),Barrier2(:,2),[0,0,0]);
fill(Barrier3(:,1),Barrier3(:,2),[0,0,0]);
set(gca, 'fontSize',16);
xlabel('X axis');
ylabel('Y axis');
title('UAV Path (1:10 cm)');
plot(position(p,1),position(p,2),'.');
grid on 
grid minor
axis equal

xlswrite('date',position);

cl=0:pi/20:2*pi;
for draw=1:p
    
    Xa=r_A*cos(cl)+position(draw,1);
    Ya=r_A*sin(cl)+position(draw,2);

    if ~ishandle(h),return,end
    plot(Xa,Ya,'r-')
    axis equal
    drawnow;
    mov(draw)=getframe;
    pause(.1)
end
 movie2avi(mov,'histogram.avi','compression','none');
 grid off
 figure
xx=5:5:360;
yy=H_s;
bb=bar(xx,yy);
xlim ([0 360]);
set(gca, 'fontSize',16);
set(gca, 'XTick',0:20:360);
xlabel('Degree');
ylabel('Density');
title('Polar Histogram');





        
    