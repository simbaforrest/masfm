function [Rba,tba]=invRt(Rab,tab)
Rba=Rab';
tba=-Rab'*tab;
end