function [v,w,t_dir,D]= fcn(pose,waypoints   )
% here whe no intersection we use L=last TP on path or E whichever is
% closer
% but in finding TP bot x , signs introduced coz for 
%right turn we need x as negative and it means TP ang is >180
%furter when ang==pi that case is handled
    flag=0;
    max_wRef=1.5;
    vRef=0.75
    t1=0
    t2=1
    wpx=waypoints(:,1);
    wpy=waypoints(:,2);
    dist_from_wp=sqrt((pose(1,1)-wpx).^2 +(pose(2,1)-wpy).^2);
    closest_wp_index=find(dist_from_wp==min(dist_from_wp));
    closest_wp=waypoints(closest_wp_index(1,1),:);
    s=size(waypoints);
    next_wp=waypoints(closest_wp_index(1,1),:);
    if(closest_wp_index(1,1)~=s(1))
        
        next_wp=waypoints(closest_wp_index(1,1)+1,:);
    end
    if(closest_wp_index(1,1)==s(1))
             closest_wp=pose(1:2,1)';
             next_wp=waypoints(closest_wp_index(1,1),:);
    end
    E=transpose(closest_wp);
    TP=E
    L=transpose(next_wp);
    last_TP_on_path=E;
    C=pose(1:2,1);
    d=L-E;
    f=E-C;
    l=0.35; %lookahead distance
    a=norm(d)^2;
    b=2*dot(f,d);
    c=norm(f)^2-l^2;
    D=b^2-(4*a*c);
    if (D<0)
        flag=1;
    end
    
    if (D>=0)
        D=sqrt(D);
        t1=(-b-D)/(2*a);
        t2=(-b+D)/(2*a);
        if( (t1<0 || t1>1) && (t2<0 || t2>1))
            flag=1;
        end
    end
    D=t2;
    if(flag==0)
        if(D==0)
            TP=E+t1*d;
        end
        if (D>0)
            if( (t1<0 || t1>1) && (t2>=0 && t2<=1))
                TP=E+t2*d;
                %disp(vv)
                
            end
            if((t1>=0 && t1<=1) && (t2<0 || t2>1))
                TP=E+t1*d;
            end
            if((t1>=0 && t1<=1) && (t2>=0 && t2<=1))
                TP1=E+t1*d;
                TP2=E+t2*d;
                TPs=[transpose(TP1);transpose(TP2)];
                TPx=TPs(:,1);
                TPy=TPs(:,2);
                dist_from_L=sqrt((L(1,1)-TPx).^2 +(L(2,1)-TPy).^2);
                closest_TP_index=find(dist_from_L==min(dist_from_L));
                TP=transpose(TPs(closest_TP_index(1,1),:));
            end
        end
        last_TP_on_path=TP
    end
    if(flag==1)
        
        norm1=norm(pose(1:2,1)-last_TP_on_path)
        norm2=norm(pose(1:2,1)-E)
        if(norm1<=norm2)
            L=last_TP_on_path;
        end
        if(norm1>norm2)
            L=E
        end
        E=pose(1:2,1);
        d=L-E;
        TP=E+l*(d)/norm(d);
        %TP=last_TP_on_path
    end
    TP_bot_x=0;
    t_dir=0;
    distance_from_TP=norm(pose(1:2,1)-TP);
    vv=TP-pose(1:2,1);
    phi=atan2(vv(2,1),vv(1,1));
    if(phi<0)
        phi=2*pi+phi;
    end
    ang=phi-(sign(pose(3,1)))*pose(3,1);
    if(ang<0)
        ang=2*pi+ang;
    end
    if(ang>pi)
        TP_bot_x=-l*cos((pi/2)+(ang-2*pi));
    end
    if(ang<=pi)
        TP_bot_x=l*cos((pi/2)-ang);
        
    end
    disp(pose(3,1))    
    
    %TP_bot_x=sign(phi)*distance_from_TP*cos(abs(phi)+sign(phi)*((pi/2)-pose(3,k-1)));
    vRef=0.75
    R=(l^2)/(2*(TP_bot_x));
    delta=atan(2*TP_bot_x*(R-TP_bot_x)/(l*l));
    wRef=(vRef)/(R);
    if(ang==pi)
        wRef=max_wRef
        vRef=0
    end
    %disp(wRef)
     if(abs(wRef)>max_wRef)
         wRef=sign(wRef)*max_wRef;
     end
    flag=0;
    w=wRef
    v=vRef
    t_dir=phi-pose(3,1);
    if(isnan(t_dir))
        t_dir=0;
    end
%     v=0;
%     w=-0.5;
%     t_dir=-1
end
    
    
    






