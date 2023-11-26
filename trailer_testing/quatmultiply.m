function q = quatmultiply(q1, q2)
    % Extract the components of the first quaternion
    comp1 = compact(q1);
    w1 = comp1(1);
    x1 = comp1(2);
    y1 = comp1(3);
    z1 = comp1(4);

    % Extract the components of the second quaternion
    comp2 = compact(q2);
    w2 = comp2(1);
    x2 = comp2(2);
    y2 = comp2(3);
    z2 = comp2(4);

    % Compute the scalar part of the product
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2;

    % Compute the vector part of the product
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    y = w1*y2 + y1*w2 + z1*x2 - x1*z2;
    z = w1*z2 + z1*w2 + x1*y2 - y1*x2;

    % Create the resulting quaternion
    q = quaternion(w, x, y, z);
end