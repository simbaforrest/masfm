function r=randin(a,b,varargin)
% generate uniform random numbers in [a,b]
if a>b
  t=a;  a=b;  b=t; %swap
end
r=a+(b-a)*rand(varargin{:});
end