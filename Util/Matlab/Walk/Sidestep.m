function [ Ge ] = Sidestep( s, struct )
%SIDESTEP Führt einen Ausfallschritt aus.
% Idee: Controller soll nach Störung möglichst zum gleichem ZMP wie im umgestörtem Fall führen: C*(^x(k+2)-x(k+2))=-C*e(k), abzüglich eines Ausgleichs. Aber |C*x(k+2)-y_d(k+2)|<e, also muss auch gelten |C*^x(k+2)-(y_d(k+2)+y_m(k+2))|<e
% Zu zeigen dass Stabil bleibt über inkrementellem Fehler in open und close loop case
    
   Ge = (struct.c0 *struct.b0 * sum(struct.Gd(s:struct.N)))^-1 ...
        * struct.c0 * (struct.A0 + eye(3) ...
        -struct.b0 * struct.Gi * struct.c0 ...
        -struct.b0 * struct.Gx);
end

