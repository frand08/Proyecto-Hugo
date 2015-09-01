N=10000;
t=1:N;
measurements = (t)  + 10*randn(1,N); % Angulo entregado por el accel
measurements = measurements';
N = length(measurements);

dt = .01; % Tiempo entre las mediciones
 
u = 0;

x = [0 ; 0]; % Estado inicial (alfa, bias -> valor del giroscopio en estado estacionario)
P = [1000 0 ; 0 1000]; % Incertidumbre inicial (aca no se cual seria)
B = [1 ; 0];
F = [1 -dt ; 0 1]; % Funcion del proximo estado
H = [1 0]; % Funcion de measurement

% Valor de la varianza falta para R
R = [.0001]; % Incertidumbre de la measurement
%

I = [1 0 ; 0 1]; % Matriz identidad
for i=1:N
    % Measurement Update
    Z = measurements(i,:);
    Y = Z - H*x;
    S = H*P*H' + R;
    K = P*H'*(inv(S));
    x = x + (K*Y);
    P = (I - (K*H))*P;
    b(i)=x(1);                          % ACA TENGO QUE TOMAR
    % Prediction
    x = (F*x + B*u);
    a(i)=x(1);                          % ACA NO!!! SERIA LA PROXIMA ESTA
    P = F*P*F';
    
end
subplot(211);
plot(b);
subplot(212);
plot(a);