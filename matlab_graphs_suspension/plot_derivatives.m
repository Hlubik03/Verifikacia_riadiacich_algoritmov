function plot_derivatives(expression, interval)
syms x;
df1 = matlabFunction(diff(expression(x),x));
ddf1 = matlabFunction(diff(df1(x),x));

figure
grid on
box on
hold on
p(1) = plot(interval,expression(interval));
p(2) = plot(interval,df1(interval));
p(3) = plot(interval,ddf1(interval));
set(p,'linewidth',2)
legend('f(x)','f''(x)')
xlabel('x')
ylabel('f(x)')

end