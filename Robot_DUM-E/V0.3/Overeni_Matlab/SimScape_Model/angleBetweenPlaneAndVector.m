function theta = angleBetweenPlaneAndVector(A, B, C, P)
    % Vstup: A, B, C jsou body definující rovinu, P je bod definující vektor
    % Výstup: theta - úhel mezi rovinou a vektorem v radiánech

    % Vektory v rovin?
    v1 = B - A;
    v2 = C - A;

    % Normálový vektor k rovin?
    n = cross(v1, v2);

    % Vektor definovaný bodem P a po?átkem
    v = P;

    % Kosinus úhlu
    cos_theta = dot(n, v) / (norm(n) * norm(v));

    % Úhel mezi normálou roviny a vektorem
    theta = pi/2 - acos(abs(cos_theta));  % Abs zajistí rozsah úhlu [0, pi/2]
end