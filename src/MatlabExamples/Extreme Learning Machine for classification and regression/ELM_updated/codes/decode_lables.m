function [NBY,NBYts]=decode_lables(BY,BYts,label)
% Note  : instances must be a colomn vectors
% BY    : binary representation of estimated training Targets
% BYts  : binary representation of estimated testing Targets
% lable : Vector contains original non binary lables
% NBY   : non binary training lables 
% NBYts : non binary testing lables 
[V,Its]=max(BYts,[],1);
NBYts=label(Its);
[V,I]=max(BY,[],1);
NBY=label(I);
end