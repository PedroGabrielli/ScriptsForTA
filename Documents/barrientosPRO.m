% ===============================================================
% SCRIPT GENÉRICO DE CINEMÁTICA DIRECTA (DENAVIT-HARTENBERG)
%
% Qué hace:
% 1. Define un robot usando una tabla DH paramétrica (simbólica).
% 2. Calcula la matriz de transformación homogénea total T (simbólica).
% 3. Sustituye valores numéricos en las variables de articulación.
% 4. Muestra la matriz T numérica y la posición (x,y,z) resultante.
%
% Requiere: Symbolic Math Toolbox
% ===============================================================
clear; clc;

% ------------------------------------------------------------------
% PARTE 1: DEFINIR VARIABLES SIMBÓLICAS
% ------------------------------------------------------------------
% Edita esta línea para incluir TODAS las variables de tus articulaciones
% (Ej. q1, q2, q3, d1, d4, etc.)
syms q1 d2 d3 q4

% ------------------------------------------------------------------
% PARTE 2: DEFINIR LA "MATRIZ" (TABLA DE PARÁMETROS DH)
% ------------------------------------------------------------------
% Edita esta tabla. Cada fila es una articulación.
% El formato de cada fila es: [theta_i, d_i, a_i, alpha_i]
%
% (Ejemplo: Robot Cilíndrico de tu ejercicio anterior)
DH_table = [
%   theta |    d    |   a   |  alpha
% ----------------------------------------
    q1,     1.2,      0,      0;      % Articulación 1
    pi/2,   d2,       0,      pi/2;   % Articulación 2
    0,      d3,       0,      0;      % Articulación 3
    q4,     1.3,      0,      0       % Articulación 4
];
% ------------------------------------------------------------------
% PARTE 3: ASIGNAR VALORES A LAS VARIABLES
% ------------------------------------------------------------------
% Lista de las variables que definiste en la PARTE 1
vars = [q1, d2, d3, q4];

% Lista de los valores numéricos que quieres asignarles
% (En el mismo orden que la lista 'vars')
values = [-pi/4, 0.8, 0.7, 0];


% ===============================================================
% MOTOR GENÉRICO (No necesitas editar de aquí en adelante)
% ===============================================================
fprintf('Calculando Cinemática Directa...\n');

% Inicializar la Matriz de Transformación Total
n_joints = size(DH_table, 1);
T_total = eye(4);
A_matrices = cell(n_joints, 1); % Para guardar cada matriz Ai

% Bucle para construir y multiplicar las matrices
for i = 1:n_joints
    % Extraer parámetros DH de la fila 'i'
    theta_i = DH_table(i, 1);
    d_i     = DH_table(i, 2);
    a_i     = DH_table(i, 3);
    alpha_i = DH_table(i, 4);

    % Construir la matriz Ai usando la función local
    A_i = dh_matrix(theta_i, d_i, a_i, alpha_i);

    % Guardar y multiplicar
    A_matrices{i} = A_i;
    T_total = T_total * A_i;
end

% Simplificar la matriz simbólica (puede tardar un poco)
fprintf('Simplificando T_total (simbólica)...\n');
T_total = simplify(T_total);

% -------------------- RESULTADOS -------------------------------
fprintf('\n================== RESULTADOS ==================\n');

% 1. Mostrar la Matriz Simbólica
fprintf('Matriz de Transformación Total (Simbólica):\n');
disp(T_total);

% 2. Sustituir valores y mostrar Matriz Numérica
fprintf('Sustituyendo valores:\n');
disp(vars);
disp(values);

T_numeric = subs(T_total, vars, values);
T_numeric = double(T_numeric); % Convertir de simbólico a numérico

% Limpiar ceros muy pequeños (errores de punto flotante)
tol = 1e-12;
T_numeric(abs(T_numeric) < tol) = 0;

fprintf('\nMatriz de Transformación Total (Numérica):\n');
disp(T_numeric);

% 3. Extraer la posición final
P_final = T_numeric(1:3, 4); % Tomar las 3 primeras filas de la 4ta columna

fprintf('\nPosición final (x, y, z):\n');
fprintf('x = %.6f\n', P_final(1));
fprintf('y = %.6f\n', P_final(2));
fprintf('z = %.6f\n', P_final(3));
fprintf('==============================================\n');


% --- FUNCIÓN AUXILIAR DE DENAVIT-HARTENBERG ---
function A_i = dh_matrix(theta, d, a, alpha)
    % Esta función construye una matriz de transformación homogénea
    % A(i-1) -> (i) a partir de sus 4 parámetros DH
    
    c_th = cos(theta);
    s_th = sin(theta);
    c_al = cos(alpha);
    s_al = sin(alpha);
    
    A_i = [
        c_th,   -s_th*c_al,   s_th*s_al,    a*c_th;
        s_th,   c_th*c_al,    -c_th*s_al,   a*s_th;
        0,      s_al,         c_al,         d;
        0,      0,            0,            1
    ];
end