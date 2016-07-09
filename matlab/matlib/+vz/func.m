function h=func(hf, x0, x1, nsamples, varargin)
% visualize a scalar function hf from x0 to x1 along direction x1-x0
% hf<handle,Rn->R>: the scalar function handle
% x0<nx1>: starting point
% x1<nx1>: ending point
% nsamples[100,1x1]: number of samples/points

if nargin<4; nsamples=100; end;
T=linspace(0,1,nsamples);
tf=@(t) hf(x0*(1-t)+x1*t);
F=arrayfun(tf,T);
h=plot(T,F,varargin{:});
end