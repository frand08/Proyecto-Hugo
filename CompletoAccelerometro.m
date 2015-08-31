clear all
close all


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AuxComplementarioYx = 0;
AuxComplementarioYy = 0;
AuxComplementarioYz = 0;
ContadorCero = 0;
TIEMPO = 0;
tiempo = 0;
cont = 0;
contadorInicio = 0;
contadorFactores = 0;
Yx=0;
Yy=0;
Yz=0;
vectorImpPos = 0;
vectorImpVel = 0;
vectorImpAcc = 0;
vectorImpAngZvert = 0;
vectorImpAngZHor = 0;
vectorImpAngYVer = 0;
vectorImpAngXVer = 0;
vectorX = 0;
vectorY = 0;
vectorZ = 0;
flag_negativo = 0;
contadortotal = 0;
zant=0;  
velant=0 ;
velact = 0 ;
posant=0;
posact=0;
signoTrapezioA=0;
signoTrapezioV=0;
limitePlotX = NUMEROFACTORES*PASO;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%defines
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
OFFSET_ACC_X = -196;
OFFSET_ACC_Y = 237;
OFFSET_ACC_Z = 233;
OFFSET_GYR_X = 135;
OFFSET_GYR_Y = 87;
OFFSET_GYR_Z = -77;
NUMEROFACTORES = 100;
NIVEL_RUIDO_POSITIVO =  0.1;
NIVEL_RUIDO_NEGATIVO = -0.1;
CantidadDeCeros = 15;
SENSIBILIDAD_ACC  = 16384;
SENSIBILIDAD_GYRO = 131.072;
ALPHA = 0.98;%0.45454545454545;
MILISEG_A_SEG = 1000;
G =	9.80665;
PASAJE_RAD_A_DEG = (180/pi);
PASAJE_DEG_A_RAD = (pi/180);
PASO = 0.01;

PASAR_A_MS2	= G / SENSIBILIDAD_ACC;
dtC = 10/MILISEG_A_SEG; %tiempo milisegundo para tomar muestra
GYRO_MULT =(1/SENSIBILIDAD_GYRO); %sensibilidad del gyroscopo en el chip

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%FILTRO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Choose filter cutoff frequency (6 kHz)
cutoff_hz = 6;
 
% Normalize cutoff frequency (wrt Nyquist frequency)
sample_rate = 100;
nyq_freq = sample_rate / 2;
cutoff_norm = cutoff_hz / nyq_freq;
 
% FIR filter order (i.e. number of coefficients - 1)
order = 50;


fir_coeff = fir1(order, cutoff_norm);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s= serial('COM15');
set(s,'Baudrate',19200); % se configura la velocidad a 19200 Baudios
set(s,'StopBits',1); % se configura bit de parada a uno
set(s,'DataBits',8); % se configura que el dato es de 8 bits, debe estar entre 5 y 8
set(s,'Parity','none'); % se configura sin paridad
%set(s,'Terminator','$');% “c” caracter con que finaliza el envío 
set(s,'InputBufferSize',2); % ”n” es el número de bytes a recibir
set(s,'Timeout',15); % 5 segundos de tiempo de espera



fopen(s);

while contadortotal < 1000
    
    a = fread(s, 1, 'int16' );
    if a == 1 %recibi encabezado
        ax = fread(s, 1, 'int16' );
        ay = fread(s, 1, 'int16' );
        az = fread(s, 1 ,'int16');
        gx = fread(s, 1 ,'int16');
        gy = fread(s, 1 ,'int16' );
        gz = fread(s, 1 ,'int16' );
        
        a = fread(s, 1 ,'int16' );
        if a == -1   %recibi el fin
            %Si llego aca es porque recibi bien la trama
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Paso cuenta a valores
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ax = ax + OFFSET_ACC_X;
            ay = ay + OFFSET_ACC_Y;
            az = az + OFFSET_ACC_Z;
            gx = gx + OFFSET_GYR_X;
            gy = gy + OFFSET_GYR_Y;
            gz = gz + OFFSET_GYR_Z;
            
            AccX=ax*PASAR_A_MS2;
            AccY=ay*PASAR_A_MS2;
            AccZ=az*PASAR_A_MS2;
            GyrX=gx*GYRO_MULT*dtC;  %convierte la la velocidad angular a º/seg // 2^16 / 500 = 131.072
            GyrY=gy*GYRO_MULT*dtC;  %convierte la la velocidad angular a º/seg // 2^16 / 500 = 131.072
            GyrZ=gz*GYRO_MULT*dtC;  %convierte la la velocidad angular a º/seg // 2^16 / 500 = 131.072

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%mantengo un largo de vector para que no crezca infinitamente
            if contadorFactores < NUMEROFACTORES
                contadorFactores = contadorFactores + 1;
                vectorX = [vectorX,AccX];
                vectorY = [vectorY,AccY];
                vectorZ = [vectorZ,AccZ];
            else
                vectorX = [vectorX(2:end),AccX];
                vectorY = [vectorY(2:end),AccY];
                vectorZ = [vectorZ(2:end),AccZ];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%
            %filtro la senial para tener la continua en esto.
            filtered_signalX = filter(fir_coeff, 1, vectorX);
            filtered_signalY = filter(fir_coeff, 1, vectorY);
            filtered_signalZ = filter(fir_coeff, 1, vectorZ);
            
            
            if contadorInicio < 210
                contadorInicio = contadorInicio + 1;
                if contadorInicio == 100
                    %%hacer un varias de estas para sacar un calculo de glocal
                    %gLocal = (-1) * sqrt(filtered_signalX(end)*filtered_signalX(end)+filtered_signalY(end)*filtered_signalY(end)+filtered_signalZ(end)*filtered_signalZ(end))                    %gLocal = -9.7969;
                    gLocal = -9.820554999263957; %hardcodeado me funciono mejor
                    contadorInicio = 300;
                end
            else
                contadortotal = contadortotal + 1 %para tomar hasta una cantidad de muestras y luego graficar
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%5
                %obtengo angulos con filtro complementario
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%5
                if AccY == 0 && AccZ == 0
                    Ang_Acc_ZY = 0;
                else
                    Ang_Acc_ZY = (atan2 (AccY,AccZ));
                end
                if AccX == 0 && AccZ == 0
                    Ang_Acc_ZX = 0;
                else
                    Ang_Acc_ZX = (atan2 (AccX,AccZ));
                end
                if AccX == 0 && AccY == 0
                    Ang_Acc_YX = 0;
                else
                    Ang_Acc_YX = (atan2 (AccY,AccX));
                end

                Ang_Acc_ZY = Ang_Acc_ZY*PASAJE_RAD_A_DEG;
                Ang_Acc_ZX = Ang_Acc_ZX*PASAJE_RAD_A_DEG;
                Ang_Acc_YX = Ang_Acc_YX*PASAJE_RAD_A_DEG; %lo paso a deg porque el gyroscopo lo da en deg


                if (Ang_Acc_YX < 0) % Con esto evito el salto en 180° y en 360°
                    Ang_Acc_YX = -Ang_Acc_YX;
                    flag_negativo=1;
                end

                AuxComplementarioYx= ALPHA*(GyrX+AuxComplementarioYx)+(1-ALPHA)*Ang_Acc_ZY;
                AuxComplementarioYy= ALPHA*(GyrY+AuxComplementarioYy)+(1-ALPHA)*Ang_Acc_ZX;
                AuxComplementarioYz= ALPHA*(GyrZ+AuxComplementarioYz)+(1-ALPHA)*Ang_Acc_YX;

                Ang_Acc_ZY = AuxComplementarioYx;
                Ang_Acc_ZX = AuxComplementarioYy;
                Ang_Acc_YX = AuxComplementarioYz;

                if (flag_negativo == 1) %Con esto evito el salto en 180° y en 360°
                        Ang_Acc_YX = -Ang_Acc_YX;
                        flag_negativo=0;
                end

                AuxAccY= AccZ * tan(Ang_Acc_ZY/(PASAJE_RAD_A_DEG));
                AuxAccX= AccZ * tan(Ang_Acc_ZX/(PASAJE_RAD_A_DEG));
                AuxAccZ= AccZ;

                %Pasa decimales a entero según ANGLE_PRECISION
                Acc_X2 = AuxAccX*AuxAccX;
                Acc_Y2 = AuxAccY*AuxAccY;
                Acc_Z2 = AuxAccZ*AuxAccZ;

                % Cilindricas Z
                %anguloVerticalZ = atan2(sqrt(Acc_X2+Acc_Y2),AuxAccZ)*PASAJE_RAD_A_DEG *ANGLE_PRECISION;
                anguloVerticalZ = atan2(sqrt(Acc_X2+Acc_Y2),AuxAccZ);
                % Obtiene ángulo respecto al eje X a partir de los datos del acelerómetro (Coord Esféricas Phi)
                % valor en grados
                anguloHorizontalZ = Ang_Acc_YX;
                % Cilindricas Y
                %anguloVerticalY = atan2(sqrt(Acc_X2+Acc_Z2),AuxAccY)*PASAJE_RAD_A_DEG *ANGLE_PRECISION;
                anguloVerticalY = atan2(sqrt(Acc_X2+Acc_Z2),AuxAccY);
                % Cilindricas X
                %anguloVerticalX = atan2(sqrt(Acc_Y2+Acc_Z2),AuxAccX)*PASAJE_RAD_A_DEG *ANGLE_PRECISION;
                anguloVerticalX = atan2(sqrt(Acc_Y2+Acc_Z2),AuxAccX);
                

                %guardo el vector de valores por si quiero graficarlo
                vectorImpAngZvert = [vectorImpAngZvert,anguloVerticalZ*PASAJE_RAD_A_DEG];
                vectorImpAngZHor = [vectorImpAngZHor,anguloHorizontalZ];
                vectorImpAngYVer = [vectorImpAngYVer,anguloVerticalY*PASAJE_RAD_A_DEG];
                vectorImpAngXVer = [vectorImpAngXVer,anguloVerticalX*PASAJE_RAD_A_DEG];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %Termino de obtener angulos
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %Paso a ejes fijo
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                AccZ = -filtered_signalZ(end);
                AccY = -filtered_signalY(end);
                AccX = -filtered_signalX(end);
                %lo que influye z
                zTrasladado = AccZ * cos(anguloVerticalZ);
                %lo que influye y
                zTrasladado = zTrasladado + AccY * cos(anguloVerticalY);
                %lo que influye x
                zTrasladado = zTrasladado + AccX * cos(anguloVerticalX);

                zTrasladado = zTrasladado - gLocal;
                if zTrasladado<NIVEL_RUIDO_POSITIVO && zTrasladado> NIVEL_RUIDO_NEGATIVO
                    zTrasladado = 0;
                end
                zTrasladado = -zTrasladado;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %Termino pasaje a ejes fijos
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %doble integral
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                if((zTrasladado > 0 && (zant < zTrasladado) ) || (zTrasladado < 0 && (zant > zTrasladado)))
                    signoTrapezioA = 1;
                end
                if((zTrasladado > 0 && (zant > zTrasladado) ) || (zTrasladado < 0 && (zant < zTrasladado)))
                    signoTrapezioA = -1;
                end
                if((zTrasladado < 0 && zant > 0) || (zTrasladado > 0 && zant < 0))
                     signoTrapezioA = 0;
                end
                   

                %// va contando la cantidad de muestras en 0 de la acc
                if( zTrasladado ==0.00 )
                     ContadorCero = ContadorCero +1;
                else
                     ContadorCero=0;
                end
                
                 if(ContadorCero>=CantidadDeCeros)  %//si llega a un límite, suponemos que se detuvo y hacemos vel=0
                 
                     velant = 0;
                     velact = 0;
                 
                 end

                %// 1ra integral: sumatoria de area(aprox trapezoidal), suponiendo Tsample=1 y que se mantiene cte.)
                velact = velant + zTrasladado*PASO; %+ signoTrapezioA*(zTrasladado-zant)*(PASO)/2;

                if((velact > 0 && (velant < velact) ) || (velact < 0 && (velant > velact)))
                    signoTrapezioV = 1;
                end
                if((velact > 0 && (velant > velact) ) || (velact < 0 && (velant < velact)))
                    signoTrapezioV = -1;
                end
                if((velact < 0 && velant > 0) || (velact > 0 && velant < 0))
                    signoTrapezioV = 0;
                end
                    
                %// 2da integral
                posact = posant + velact*PASO + signoTrapezioV*(velact-velant)*PASO/2 ;

                zant=zTrasladado;  %// paso la acc actual a la acc anterior
                velant=velact; % // lo mismo para la velocidad y la posicion
                posant=posact;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %Imprimo
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                
                vectorImpPos = [vectorImpPos,posact];
                vectorImpVel = [vectorImpVel,velact];
                vectorImpAcc = [vectorImpAcc,zTrasladado];
                %tiempo= [tiempo,tiempo(end)+PASO];
                
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %Termino de imprimir
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                
              
            end
        end
    end
end
fclose(s);
 delete(s);
 clear s;
 
gLocal = gLocal

figure(1)
subplot(3,1,1)
plot(vectorImpAcc)
title('accelerom')

subplot(3,1,2)
plot(vectorImpVel)
title('velocidad')

subplot(3,1,3)
plot(vectorImpPos)
title('posicion')
% 
% figure(2)
% subplot(2,2,1)
% plot(vectorImpAngZvert)
% title('Z vertical')
% 
% subplot(2,2,2)
% plot(vectorImpAngZHor)
% title('Z horizontal')
% 
% subplot(2,2,3)
% plot(vectorImpAngYVer)
% title('Y vertical')
% 
% subplot(2,2,4)
% plot(vectorImpAngXVer)
% title('X vertical')

 