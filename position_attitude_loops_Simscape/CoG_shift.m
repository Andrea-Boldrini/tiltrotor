clc

Lh = 0.2;
pitch = [0 15 30 45 60];
Tf = [2.75 2.72 2.69 2.66 2.59];
Tb = [2.75 2.77 2.8 2.84 2.9];

for i = 1 : length(pitch)
    shift(i) = Lh*(1-Tf(i)/Tb(i))/(1+Tf(i)/Tb(i));
end

p1 = polyfit(pitch, shift, 1); % Linear fit
p2 = polyfit(pitch, shift, 2); % Quadratic fit
p3 = polyfit(pitch, shift, 4); % Quadratic fit
x = 0:1:60;
y_fit1 = polyval(p1, x);
y_fit2 = polyval(p2, x);
y_fit3 = polyval(p3, x);

lineWidth = 1.4;
figure
plot(x, y_fit1, 'LineWidth', lineWidth)
hold on
plot(x, y_fit2, 'LineWidth', lineWidth)
hold on
plot(x, y_fit3, 'LineWidth', lineWidth)
hold on
plot(pitch, shift, 'o', 'LineWidth', lineWidth)
xlabel('Pitch (deg)', 'FontSize', 12)
ylabel('Shift of CoG', 'FontSize', 12)
title('Shift of CoG vs. Pitch', 'FontSize', 14)
legend("p1", "p2", "p4", "samples", "Location", "best", 'FontSize', 10)
grid on
