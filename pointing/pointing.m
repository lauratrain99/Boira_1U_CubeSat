
title("Data gathered during one year")
set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

clear;clc;

mag = 104.99*2;



a = linspace(0,2*pi,52);



for i = 1:150
    xcircle(i,:) = mag/2*sin(a);
    ycircle(i,:) = mag/2*cos(a) + (i-1)*1.397;
    r(i,:) = normrnd(0,mag/2/3 ,[1,52]);
    v(i,:) = normrnd(0,mag/2/3 ,[1,52]) + (i-1)*1.397;
    
end

for i = 151:300
    xcircle(i,:) = mag/2*sin(a);
    ycircle(i,:) = mag/2*cos(a) - ((i-150)-1)*1.397;
    r(i,:) = normrnd(0,mag/2/3 ,[1,52]);
    v(i,:) = normrnd(0,mag/2/3 ,[1,52]) - ((i-150)-1)*1.397;
    
end

for i = 1:300
    plot(r(i,:),v(i,:),'r*')
    hold on
%     plot(xcircle(i,:),ycircle(i,:),'b')
    hold on
    axis equal
end

count = 1;
for i = 1:300
    for j = 1:52
        if r(i,j) < max(xcircle(1,:)) && r(i,j) > min(xcircle(1,:)) && v(i,j) < max(ycircle(1,:)) && v(i,j) > min(ycircle(1,:))
            count = count + 1;
        end
    end
end

plot(xcircle(1,:),ycircle(1,:),'b','LineWidth',2)
xlabel("[km]")
ylabel("[km]")
title("Data gathered during one year")
axis equal

% text(75,20, " \~ 4000 points")