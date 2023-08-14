function [SteerDir,op,op1]= fcn(Ranges,Angles,pose,rmax,TargetDir)




global log_odds;



op1=zeros(1,1);
op=zeros(1,1);

global theta_steer_prev;
res=0.1;
w_s=25;
coder.varsize('Start');
coder.varsize('End');
coder.varsize('Start');
coder.varsize('temp4');

xx=1;
p=0;
yy=1;
xend=1;
yend=1;

TargetDir=robotics.internal.wrapToPi(TargetDir);
safety=2.0;
for k=1:numel(Ranges)
    dist=Ranges(k);
     if isnan(Ranges(k))  
         dist=double(rmax);
     end
     theta=Angles(k);
     if(pose(3,1)>=0)
     if(theta<=0)
         theta=pose(3,1)-abs(theta);
     end
     if(theta>0)
         theta=pose(3,1)+theta;
     end
     end
     if(pose(3,1)<0)
     if(theta<=0)
         theta=pose(3,1)+(theta);
     end
     if(theta>0)
         theta=-abs(pose(3,1))+theta;
     end
     end
     %theta=robotics.internal.wrapToPi(theta);  
%      if(theta>pi)
%          theta=theta-2*pi;
%      end
%      theta=(pi/2-theta);
%      theta=-1*theta;    
     
     ang=theta;
     
     
     if(theta>=0 && theta<pi/2)
             xx=pose(1,1):(res/2)*cos(ang):min(pose(1,1)+(dist-2*res)*cos(ang),12.5);
             yy=pose(2,1):(res/2)*sin(ang):min(pose(2,1)+(dist-2*res)*sin(ang),12.5);
             
             if(ang==0)
                 yy=ones(1,numel(xx))*pose(2,1);
             end
             xx=floor(xx/res)+1.0;
             yy=floor(yy/res)+1.0;
             xend=min(pose(1,1)+(dist)*cos(ang),12.5);
             yend=min(pose(2,1)+(dist)*sin(ang),12.5);
             xend=floor(xend/res)+1.0;
             yend=floor(yend/res)+1.0;
             
     end
     if(theta<=pi && theta>=pi/2)
              ang=pi-theta;
             xx=pose(1,1):-(res/2)*cos(ang):max(0,pose(1,1)-(dist-2*res)*cos(ang));
             yy=pose(2,1):(res/2)*sin(ang):min(pose(2,1)+(dist-2*res)*sin(ang),12.5);
             if(ang==0)
                 yy=ones(1,numel(xx))*pose(2,1);
             end
             if(ang==pi/2)
                 xx=ones(1,numel(yy))*pose(1,1);
             end
             xx=floor(xx/res)+1.0;
             yy=floor(yy/res)+1.0;
             xend=max(pose(1,1)-(dist)*cos(ang),0);
             yend=min(pose(2,1)+(dist)*sin(ang),12.5);
             xend=floor(xend/res)+1.0;
             yend=floor(yend/res)+1.0;
     end
          if( theta>=-pi && theta<=-pi/2)
              ang=pi-abs(theta);
             xx=pose(1,1):-(res/2)*cos(ang):max(0,pose(1,1)-(dist-2*res)*cos(ang));
             yy=pose(2,1):-(res/2)*sin(ang):max(0,pose(2,1)-(dist-2*res)*sin(ang));
             if(ang==0)
                 yy=ones(1,numel(xx))*pose(2,1);
             end
             if(ang==pi/2)
                 xx=ones(1,numel(yy))*pose(1,1);
             end
             xx=floor(xx/res)+1.0;
             yy=floor(yy/res)+1.0;
             xend=max(pose(1,1)-(dist)*cos(ang),0);
             yend=max(pose(2,1)-(dist)*sin(ang),0);
             xend=floor(xend/res)+1.0;
             yend=floor(yend/res)+1.0;
          end
          if( theta<0 && theta>-pi/2)
              ang=abs(theta);
             xx=pose(1,1):(res/2)*cos(ang):min(pose(1,1)+(dist-2*res)*cos(ang),12.5);
             yy=pose(2,1):-(res/2)*sin(ang):max(0,pose(2,1)-(dist-2*res)*sin(ang));
             xx=floor(xx/res)+1.0;
             yy=floor(yy/res)+1.0;
             xend=min(pose(1,1)+(dist)*cos(ang),12.5);
             yend=max(pose(2,1)-(dist)*sin(ang),0);
             xend=floor(xend/res)+1.0;
             yend=floor(yend/res)+1.0;
          end
     
     
     for i=1:min(numel(xx),numel(yy))
         ux=xx;
         uy=yy;
         if(yy(i)>size(log_odds,1))
             uy(i)=size(log_odds,1);
             yy=uy;
         end
         if(xx(i)>size(log_odds,1))
             ux(i)=size(log_odds,1);
             xx=ux;
         end
         log_odds(yy(i),xx(i))=log_odds(yy(i),xx(i))+log(0.2/0.8);
     end
     if(~isnan(Ranges(k)))
         if(yend>size(log_odds,1))
             yend=size(log_odds,1);
         end
         if(xend>size(log_odds,1))
             xend=size(log_odds,1);
         end
         log_odds(yend,xend)=log_odds(yend,xend)+log(0.8/0.2);
     end
     

end
prob=exp(log_odds)./(1+exp(log_odds));


i1_grid=min(floor(pose(2,1)/res)+1+(w_s-1)/2,(12.5/res)+1);
j1_grid=max(floor(pose(1,1)/res)+1-(w_s-1)/2,1);
i2_grid=max(floor(pose(2,1)/res)+1-(w_s-1)/2,1);
j2_grid=min(floor(pose(1,1)/res)+1+(w_s-1)/2,(12.5/res)+1);
i1=i2_grid;
i2=i1_grid;
j1=j1_grid;
j2=j2_grid;
alpha=5*pi/180;
b1=5;
a1=b1*(2^0.5)*(w_s-1)/2;
%C_active_prob=prob(i1:i2,j1:j2);
sector_POD=zeros(1,(2*pi/alpha));
for i=i1:i2
    for j =j1:j2
        xi=(j*res)+res/2;
        yj=(i*res)+res/2;
        cell_pos=[xi;yj];
        beta_ij=atan2(cell_pos(2,1)-pose(2,1),cell_pos(1,1)-pose(1,1));
        if beta_ij<0
            beta_ij=2*pi+beta_ij;
        end
        sector=floor(beta_ij/alpha)+1;
        d=sqrt((cell_pos(1,1)-pose(1,1))^2 + (cell_pos(2,1)-pose(2,1))^2);
        if((prob(i,j)>=0.5))
        d=max(sqrt((cell_pos(1,1)-pose(1,1))^2 + (cell_pos(2,1)-pose(2,1))^2)-safety,0);
        end
        sector_POD(1,sector)=sector_POD(1,sector)+((prob(i,j)^2)*(a1-b1*d));
    end
end
l=5;
smoothened_sector_POD=zeros(1,(2*pi/alpha));
temp4=[0];
for k =1:(2*pi/alpha)
    k1=k-l;
    if(k1<1)
        k1=(2*pi/alpha)+k1;
    end
    k2=k+l;
        if(k2>(2*pi/alpha))
            k2=k2-(2*pi/alpha);
        end
     temp1=1:l+1;
     temp2=fliplr(temp1);
     temp3=cat(2,temp1,temp2(1,2:end));
     disp(k1)
     disp(k2)
     if(k1<k2)
        temp4=sector_POD(1,k1:k2).*temp3;
     end
     if(k1>k2)
         
         temp4=cat(2,sector_POD(1,k1:end),sector_POD(1,1:k2)).*temp3;
         
     end
     disp(temp4)   
     smoothened_sector_POD(1,k)=sum(temp4)/(2*l+1);
end
        
Start=[0];
End=[0];
flag=1;

opp=[min(smoothened_sector_POD);max(smoothened_sector_POD)];
h_t=1; %Histogram Threshold
 for i=1:size(smoothened_sector_POD,2)
     
     
     if(smoothened_sector_POD(i)<h_t && flag==1)
         Start=cat(2,Start,[i]);
         flag=0;
     end
     if(smoothened_sector_POD(i)>=h_t && i>1 && flag==0)
         End=cat(2,End,[i-1]);
         flag=1;
     end
     if(i==size(smoothened_sector_POD,2) && smoothened_sector_POD(i)<h_t)
         if(size(Start,2)==2 && size(End,2)==1)
             End=cat(2,End,[i]);
             continue;
         end
         if(smoothened_sector_POD(1)<h_t)
         Start(1)=Start(end);
         Start(end)=[];
         end
         if(smoothened_sector_POD(1)>=h_t)
       
        End=cat(2,End,[i]);
         end
     end
 end 
Start=Start(1,2:end);
End=End(1,2:end);
op1=size(Start,2);
op=size(End,2);
theta_targ=pose(3,1)+TargetDir;
if(theta_targ<0)
    theta_targ=2*pi+theta_targ;
end
k_targ=floor(theta_targ/alpha)+1;
Cost=inf;
kn=1;
kf=1;
smax=15;
a=3;
b=0.02;
c=0;
theta_steer_opt=TargetDir;
for m=1:size(Start,2)
    v_start=0;
    v_end=0;
    if(Start(m)<End(m))
        if(k_targ>=Start(m) && k_targ<=End(m))
            kn=k_targ;
            kf=min(kn+smax,End(m));
            if(kf>2*pi/alpha)
            kf=mod(kf,2*pi/alpha);
            end
            
            
        end
        if(~(k_targ>=Start(m) && k_targ<=End(m)))
            if(Start(m)-k_targ<0)
                v_start=2*pi/alpha;
            end
            if(k_targ-End(m)<0)
                v_end=2*pi/alpha;
            end
            if(abs(Start(m)-k_targ+v_start)<=abs(k_targ-End(m)+v_end))
                kn=Start(m);
                kf=min(kn+smax,End(m));
            if(kf>2*pi/alpha)
            kf=mod(kf,2*pi/alpha);
            end
               
                
            end
            if(abs(Start(m)-k_targ+v_start)>abs(k_targ-End(m)+v_end))
                kn=End(m);
                kf=max(kn-smax,Start(m));
            if(kf<0)
            kf=mod(kf,2*pi/alpha);
            end
                %kf=mod(kn-smax,2*pi/alpha); %mod(-12,72)=60 as per this function!! so works
                
            end
            
        end
    end
    if(Start(m)==End(m))
        kn=Start(m);
        kf=min(kn+smax,End(m));
            if(kf>2*pi/alpha)
            kf=mod(kf,2*pi/alpha);
            end
        %kf=mod(kn+smax,2*pi/alpha);
        
    end
    if(Start(m)>End(m))
        if((k_targ>=Start(m) && k_targ<=2*pi/alpha) || (k_targ>=1 && k_targ<=End(m)))
            kn=k_targ;
            kf=min(kn+smax,End(m));
            if(kf>2*pi/alpha)
            kf=mod(kf,2*pi/alpha);
            end
            %kf=mod(kn+smax,2*pi/alpha);
           
        end
        if(~((k_targ>=Start(m) && k_targ<=2*pi/alpha) || (k_targ>=1 && k_targ<=End(m))))
             if(abs(Start(m)-k_targ)<=abs(k_targ-End(m)))
                kn=Start(m);
                kf=min(kn+smax,End(m));
            if(kf>2*pi/alpha)
            kf=mod(kf,2*pi/alpha);
            end
                %kf=mod(kn+smax,2*pi/alpha);
               
            end
            if(abs(Start(m)-k_targ)>abs(k_targ-End(m)))
                kn=End(m);
                kf=max(kn-smax,Start(m));
            if(kf<0)
            kf=mod(kf,2*pi/alpha);
            end
                %kf=mod(kn-smax,2*pi/alpha); %mod(-12,72)=60 as per this function!! so works
                
            end
        end
    end
    theta_steer=(kn+kf)*alpha/2;
    if(abs(kn-kf)>smax)
        theta_steer=(((kn+kf)/2)+(pi/alpha))*alpha;
    end
    
    %###############################%
    p=pose(3,1);
    if(pose(3,1)<0)
        p=2*pi+pose(3,1);
    end
    theta1=abs(robotics.internal.wrapToPi(theta_steer-p));
    
    theta2=abs(robotics.internal.wrapToPi(theta_targ-theta_steer));
    
    theta3=abs(robotics.internal.wrapToPi(theta_steer_prev-theta_steer));
            
    if((a*theta2 +b*theta1 +c*theta3)<=Cost)   
        Cost=(a*theta2 +b*theta1 +c*theta3);
        theta_steer_opt=robotics.internal.wrapToPi(theta_steer-p);
    end
%       if((abs(kn-kf)>smax))
%            if(sum(cat(2,smoothened_sector_POD(1:min(kn,kf)),smoothened_sector_POD(max(kn,kf):end)))<=Cost)
%           Cost=sum(cat(2,smoothened_sector_POD(1:min(kn,kf)),smoothened_sector_POD(max(kn,kf):end)));
%            
%          theta_steer_opt=robotics.internal.wrapToPi(theta_steer-p);
%            end
%            
%       
%       else  
%       if(sum(smoothened_sector_POD(min(kn,kf):max(kn,kf)))<=Cost)
%           Cost=sum(smoothened_sector_POD(min(kn,kf):max(kn,kf)));
%          theta_steer_opt=robotics.internal.wrapToPi(theta_steer-p);
%       end
%       end

     
      
          
    
        
end
SteerDir=theta_steer_opt;   
theta_steer_prev=theta_steer_opt;   
if(size(Start,2)==1 && size(End,2)==1)
    if(Start(1,1)==1 && End(1,1)==2*pi/alpha)
        SteerDir=TargetDir;
        theta_steer_prev=SteerDir;   
    end
end


        
         
     
    

end
