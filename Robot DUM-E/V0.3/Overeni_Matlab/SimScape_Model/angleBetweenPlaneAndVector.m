function theta = angleBetweenPlaneAndVector(A, B, C, P)
    % Vstup: A, B, C jsou body definuj�c� rovinu, P je bod definuj�c� vektor
    % V�stup: theta - �hel mezi rovinou a vektorem v radi�nech

    % Vektory v rovin?
    v1 = B - A;
    v2 = C - A;

    % Norm�lov� vektor k rovin?
    n = cross(v1, v2);

    % Vektor definovan� bodem P a po?�tkem
    v = P;

    % Kosinus �hlu
    cos_theta = dot(n, v) / (norm(n) * norm(v));

    % �hel mezi norm�lou roviny a vektorem
    theta = pi/2 - acos(abs(cos_theta));  % Abs zajist� rozsah �hlu [0, pi/2]
end