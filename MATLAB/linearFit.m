function [fit] = linearFit(data, n)
%fit = LINEARFIT(data, n)
%Approximates a given 2D curve with linear sections in an optimal way.
%Credits:
%    Andrey Rubshtein, http://stackoverflow.com/users/817452/andrey-rubshtein
%    Nikolai Golovchenko, http://golovchenko.org
%
%INPUT
%    data       2D curve given by list of points, array of shape (2,N).
%    n          Number of segments to approximate the curve with (int).
%
%OUTPUT
%    fit        List of linear sections boundary points, array of shape (2,n).

    x = data(1,:);
    y = data(2,:);
    %Adaptive section lengths, thanks to Andrey Rubshtein.
    %http://stackoverflow.com/questions/12556491/how-to-fit-a-curve-by-a-series-of-segmented-lines-in-matlab/12556816#12556816
    %select control points on x axis
    indexes = round(linspace(1,numel(y),n));
    derivativeApprox = diff(y(indexes));
    inverseDerivative = 1./derivativeApprox;
    weightOfSection = inverseDerivative/sum(inverseDerivative);
    totalRange = max(x(:))-min(x(:));
    sectionSize = weightOfSection.*totalRange;
    x0 = x(1) + [0 cumsum(sectionSize)];

    %Piecewise linear approximation by Nikolai Golovchenko.
    % Fit a piecewise continuous function f(x) to the pairs of data points (x,y)
    % such that the sum of squares of error is minimal.
    %
    % x0 - values of x that define ends of segments of function f(x)
    % p - end points of the segments p = f(x0)
    %
    % See also: http://golovchenko.org/docs/ContinuousPiecewiseLinearFit.pdf
    % 4-May-2004 Nikolai Golovchenko.
    numberOfParameters = length(x0);
    % separate data in segments
    j = {};
    for i = 1 : numberOfParameters - 1
       j{i} = find(x > x0(i) & x <= x0(i + 1));

       if isempty(j{i})
          error('Insufficient amount of data points');
       end

    end
    % compute the matrices corresponding to the
    % system of equations
    A = zeros(numberOfParameters, numberOfParameters);
    B = zeros(numberOfParameters, 1);
    for i = 1:numberOfParameters
       if i ~= 1
          % first sum
          A(i, i-1) = A(i, i-1) - ...
             sum((x(j{i-1}) - x0(i-1)) .* (x(j{i-1}) - x0(i))) / (x0(i) - x0(i-1)) .^ 2;
          A(i, i) = A(i, i) + ...
             sum((x(j{i-1}) - x0(i-1)) .^ 2) / (x0(i) - x0(i-1)) .^ 2;

          B(i) = B(i) + ...
             (sum(x(j{i-1}) .* y(j{i-1})) - x0(i-1) * sum(y(j{i-1}))) / (x0(i) - x0(i-1));
       end
       if i ~= numberOfParameters
          % second sum
          A(i, i) = A(i, i) + ...
             sum((x(j{i}) - x0(i+1)) .^ 2) / (x0(i+1) - x0(i)) .^ 2;
          A(i, i+1) = A(i, i+1) - ...
             sum((x(j{i}) - x0(i)) .* (x(j{i}) - x0(i+1))) / (x0(i+1) - x0(i)) .^ 2;

          B(i) = B(i) + ...
             (-sum(x(j{i}) .* y(j{i})) + x0(i+1) * sum(y(j{i}))) / (x0(i+1) - x0(i));
       end
    end
    % find the parameters
    p = A^-1 * B;
    
    %   return
    fit = [x0;p'];