% Equipment Fault Diagnosis System in B-Prolog
% Constraint Processing Theory REPORT 2

% Declare dynamic predicates for Short-Term Memory (facts and triggered rules)
:- dynamic fact/2.
:- dynamic fired/1.

% ==========================================
% Task 1 & 2: Long-Term Memory (Production Rules)
% Data structure: rule(RuleID, ConditionsList, Conclusion, Confidence, Support).
% ==========================================
rule(learned_1, [fact(temperature, high), fact(vibration, high)], fact(diagnosis, bearing_failure), 1.0, 3).
rule(learned_6, [fact(temperature, high), fact(pressure, low)], fact(diagnosis, lubrication_failure), 1.0, 2).
rule(learned_11, [fact(temperature, normal), fact(vibration, high)], fact(diagnosis, shaft_misalignment), 1.0, 2).
rule(learned_14, [fact(temperature, normal), fact(noise, normal), fact(vibration, normal)], fact(diagnosis, normal), 1.0, 2).
rule(e1, [fact(temperature, high), fact(vibration, high)], fact(machine_state, unstable), 0.9, 1).
rule(e3, [fact(pressure, low), fact(rotation, low)], fact(machine_state, poor_lubrication), 0.9, 1).

% ==========================================
% Task 3: Short-Term Memory Acquisition (Observation Parsing)
% ==========================================
set_observations(ObsList) :-
    retractall(fact(_, _)),  % Clear previous diagnostic facts (STM)
    retractall(fired(_)),    % Clear previous rule firing records
    assert_obs(ObsList).

assert_obs([]).
assert_obs([(Attr, Val)|Rest]) :-
    assertz(fact(Attr, Val)), % Assert new observations into STM
    assert_obs(Rest).

% ==========================================
% Task 4: Inference Engine (Forward Chaining)
% ==========================================
infer :-
    % 1. Pattern Matching: Find all applicable, un-fired rules and build ConflictSet
    findall(rule(ID, Conds, Conc, Conf, Supp),
            (rule(ID, Conds, Conc, Conf, Supp),
             \+ fired(ID),
             check_conditions(Conds)),
            ConflictSet),
    ConflictSet \= [], % Stop if ConflictSet is empty
    
    % 2. Conflict Resolution: Select best rule based on Confidence and Support
    select_best(ConflictSet, BestRule),
    BestRule = rule(BestID, _BestConds, BestConc, BestConf, _BestSupp),
    
    % 3. Rule Firing: Mark as fired and add conclusion to STM
    assertz(fired(BestID)),
    ( \+ call(BestConc) -> assertz(BestConc) ; true ),
    
    % 4. Print reasoning trace (Explanation capability)
    write('  Fired rule: '), write(BestID), write(' -> '), write(BestConc),
    write(' [confidence='), write(BestConf), write(']'), nl,
    
    % 5. Recursive loop until no new facts can be derived
    infer.
infer. % Termination condition

% Helper predicate: Check if all antecedent conditions are present in STM
check_conditions([]).
check_conditions([Cond|Rest]) :- call(Cond), check_conditions(Rest).

% Helper predicate: Conflict resolution strategy (prioritize Confidence, then Support)
select_best([Rule], Rule).
select_best([rule(ID, Cond, Conc, Conf, Supp) | Rest], BestRule) :-
    select_best(Rest, rule(ID2, Cond2, Conc2, Conf2, Supp2)),
    ( (Conf > Conf2 ; (Conf == Conf2, Supp >= Supp2)) ->
      BestRule = rule(ID, Cond, Conc, Conf, Supp)
    ;
      BestRule = rule(ID2, Cond2, Conc2, Conf2, Supp2)
    ).

% ==========================================
% Test Execution
% ==========================================
go :-
    write('== Diagnosis: t1 =='), nl,
    write('Observed facts: temperature=high, vibration=high, noise=unknown, pressure=normal, rotation=low'), nl,
    set_observations([(temperature, high), (vibration, high), (noise, unknown), (pressure, normal), (rotation, low)]),
    infer,
    (fact(diagnosis, D1) ->
        write('Final diagnosis: '), write(D1), nl
    ;
        write('Final diagnosis: unknown'), nl
    ),
    nl,
    write('== Diagnosis: t2 =='), nl,
    write('Observed facts: temperature=high, vibration=normal, noise=unknown, pressure=low, rotation=low'), nl,
    set_observations([(temperature, high), (vibration, normal), (noise, unknown), (pressure, low), (rotation, low)]),
    infer,
    (fact(diagnosis, D2) ->
        write('Final diagnosis: '), write(D2), nl
    ;
        write('Final diagnosis: unknown'), nl
    ).
