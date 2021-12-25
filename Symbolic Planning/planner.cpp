#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <chrono>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    // bool operator<(const GroundedCondition& rhs) const
    // {
    //     return true;
    // }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_actions()
    {

        return this->actions;
    }


    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_start_state() const
    {
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_state() const
    {
        return this->goal_conditions;
    } 

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}












/******************************MY CODE *******************************/

typedef unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> State;

struct StateHasher
{
    size_t operator()(const State& state) const
    {
        string hash_string;
        for(auto const &gc : state)
        {
            hash_string = hash_string + gc.toString();
        }
        return hash<string>{}(hash_string);
    }
};

// bool operator<(const State& rhs)
// {
//     return true;
// }

struct cmp 
{
    bool operator() (pair<double,State> lhs, pair<double,State> rhs) const 
    {
        if(lhs.first != rhs.first)
        {
            return lhs.first < rhs.first;
        }
        else
        {
            return true;
        }
        
    }
};

typedef set<pair<double,State>,cmp> OpenList; // Have to input parameter compare operator
typedef unordered_set<State,StateHasher> ClosedList;

class Node
{
    public:

    double f;
    double g;
    double h;
    State parent;
    GroundedAction ga;

    Node(double f_,double g_,double h_,State parent_,GroundedAction ga_) : f(f_),g(g_),h(h_),parent(parent_),ga(ga_) {};
};

typedef unordered_map<State,Node,StateHasher> Graph;

void generateSymbolsPermutationsWithRepetitions(vector<string> symbols, vector<string> prefix,
									int num_symbols, int num_args,vector<vector<string>>& list_perm)
{
	
	// Base case: k is 0,
	// print prefix
	if (num_args == 0)
	{
        // cout <<"In Base case if" << endl;
        list_perm.push_back(prefix);
        // for(int i = 0; i < prefix.size();i++)
        // {
        //     // cout <<"In Base case for loop " << endl;
        //     cout << prefix[i] <<",";

        // }
        // cout << std::endl;
		return;
	}

	// One by one add all characters from symbols and recursively call for k equals to k-1
	for (int i = 0; i < num_symbols; i++)
	{
        // cout << "In main for loop" << endl;
		vector<string> newPrefix;
		
		// Next character of input added
        for(int i = 0; i < prefix.size();i++)
        {
            // cout <<"In nested for loop " << endl;
            newPrefix.push_back(prefix[i]);
        }
		// newPrefix = prefix + symbols[i];
        newPrefix.push_back(symbols[i]);
		
		// k is decreased, because
		// we have added a new character
		generateSymbolsPermutationsWithRepetitions(symbols, newPrefix, num_symbols, num_args - 1,list_perm);
	}

}



void generateSymbolsPermutations(vector<string> symbols, vector<string> prefix,
									int num_symbols, int num_args,vector<vector<string>>& list_perm)
{
	
	// Base case: k is 0,
	// print prefix
	if (num_args == 0)
	{
        // cout <<"In Base case if" << endl;
        list_perm.push_back(prefix);

		return;
	}

	// One by one add all characters from symbols and recursively call for k equals to k-1
	for (int i = 0; i < num_symbols; i++)
	{
        // cout << "In main for loop" << endl;
		vector<string> newPrefix;
		
		// Next character of input added
        for(int j = 0; j < prefix.size();j++)
        {
            // cout <<"In nested for loop " << endl;
            newPrefix.push_back(prefix[j]);
        }
		// newPrefix = prefix + symbols[i];
        newPrefix.push_back(symbols[i]);

        auto new_symbols = symbols;
        new_symbols.erase(find(new_symbols.begin(),new_symbols.end(),symbols[i]));

		
		// k is decreased, because
		// we have added a new character
		generateSymbolsPermutations(new_symbols, newPrefix, num_symbols-1, num_args - 1,list_perm);
	}

}

void printSymbolPermutation(vector<string> perm)
{
    for(int i = 0; i < perm.size();i++)
    {
        cout << perm[i] << "," ;

    }
    cout << endl;
}

vector<Action*> generateAllPossibleActions(Env* env)
{
    // cout << "Inside generateAllPossibleActions function call " << endl;
    // cout << "************************************************************" << endl;
    vector<Action*> all_actions;
    auto actions = env->get_actions();
    // cout << "Number of actions is " << actions.size() << endl;
    auto symbols = env->get_symbols();
    // cout << "Number of symbols is " << symbols.size()<< endl;
    // cout << "************************************************************" << endl;

    // Converting list of symbols to vector for [] access operator
    vector<string> symbol_string;
    for(auto const& s : symbols)
    {
        symbol_string.push_back(s);
    }
    int num_symbols = symbols.size();
    for(auto const& a : actions)
    {
        // cout << "Entered For loop of Actions" << endl;
        // cout << "************************************************************" << endl;
        // cout << "The action is " << endl;
        // cout << a;
        auto temp = a;

        auto args = temp.get_args();

        // Converting list of arguments to a vector for easy access
        vector<string> args_vec;
        for(auto const& argument : args)
        {
            args_vec.push_back(argument);
        }
        // cout << "The vector of args is " << endl;
        // for(int i =-0; i < args_vec.size();i++)
        // {
        //     cout << args_vec[i] << "," ;
        // }
        // cout << endl;
        int num_args = args.size();
        auto name = temp.get_name();

        auto preconditions = temp.get_preconditions();
        // cout << "Size of preconditions is " << preconditions.size() << endl;
        auto effects = temp.get_effects();
        // cout << "Size of effects is " <<  effects.size() << endl;

        vector<string> prefix;
        vector<vector<string>> list_perm ;
        // cout << "Size of list of permutations before calling generateSymbols is " << list_perm.size() << endl;
        generateSymbolsPermutationsWithRepetitions(symbol_string,prefix,num_symbols,num_args,list_perm); 
        // cout << "Size of list of permutations AFTER calling generateSymbols is " << list_perm.size() << endl;
        // cout << "********************************************************************* " << endl;

        // Vector of all possible permutations of the symbols . (num_symbols P num_args)

        for(auto const& perm : list_perm)
        {
            // Make new list of args
            // cout << "Permutation of symbols is " ;
            // printSymbolPermutation(perm);
            // cout <<endl;
            list<string> new_args;
            for(int i = 0; i < perm.size();i++)
            {
                new_args.push_back(perm[i]);
            }
            unordered_set<Condition, ConditionHasher, ConditionComparator> new_preconditions;
            for(auto const& precond : preconditions)
            {
                // cout << "General default precondition is ";
                // cout << precond << endl;

                auto temp_predicate = precond.get_predicate();
                auto temp_truth = precond.get_truth();
                auto temp_args = precond.get_args();
                list<string> new_args_copy;
                
                for(auto const &arg:temp_args)
                {
                    int addflag = 1;
                    for(int i = 0; i < args_vec.size();i++)
                    {
                        if(arg == args_vec[i])
                        {
                            new_args_copy.push_back(perm[i]);
                            addflag = 0;
                            break; 

                        }
                    }

                    if(addflag)
                    {
                        for(auto const& sym : symbol_string)
                        {
                            if(arg == sym)
                            {
                                new_args_copy.push_back(sym);
                                break;
                            }
                        }
                    }
                }

                Condition temp_condition(temp_predicate,new_args_copy,temp_truth) ;
                // cout << "Precondition being inserted is ";
                // cout << temp_condition<<endl ;
                new_preconditions.insert(temp_condition);
            }

            unordered_set<Condition, ConditionHasher, ConditionComparator> new_effects;
            for(auto const& effect : effects)
            {
                // cout << "Effect being copied is " ;
                // cout << effect << endl;
                auto temp_predicate = effect.get_predicate();
                auto temp_truth = effect.get_truth();
                auto temp_args = effect.get_args();
                list<string> new_args_copy ;

                for(auto &arg: temp_args)
                {
                    int addflag = 1;
                    for(int i = 0; i < args_vec.size();i++)
                    {
                        if(arg == args_vec[i])
                        {
                            new_args_copy.push_back(perm[i]);
                            addflag = 0;
                            break; 

                        }
                    }
                    if(addflag)
                    {
                        for(auto const& sym : symbol_string)
                        {
                            if(arg == sym)
                            {
                                new_args_copy.push_back(sym);
                                break;
                            }
                        }
                    }
                }


                Condition temp_effect(temp_predicate,new_args_copy,temp_truth);
                // cout << "Effect being inserted is " ;
                // cout << temp_effect << endl ;

                new_effects.insert(temp_effect);
                // cout << "**************************************************************" << endl;

            }

            //Assert equality of precondoitions and effects with new_preconditions and new_efefcts



            Action* temp = new Action(name,new_args,new_preconditions,new_effects); // Create new using copy constructor
            all_actions.push_back(temp);
            // cout << "**************************************************************" << endl;
            // cout << "Size of all_actions vector is " << all_actions.size() << endl;
        }


    }

    return all_actions;

}


State generateSuccessorState(State curr_state, const Action action)
{
    auto effects = action.get_effects();
    for(auto const& effect : effects)
    {
        if(effect.get_truth() == false )
        {
            // Remove this effect from curr_state
            auto temp = GroundedCondition(effect.get_predicate(),effect.get_args(),true);
            curr_state.erase(temp);

        }
        else
        {
            // Add this effect to curr_state 
            auto temp = GroundedCondition(effect.get_predicate(),effect.get_args(),true);
            curr_state.insert(temp);
        }
    }

    return curr_state;

}


State generateSuccessorStateEDL(State curr_state, const Action action)
{
    auto effects = action.get_effects();
    for(auto const& effect : effects)
    {
        if(effect.get_truth() == false )
        {
            // Do nothing

        }
        else
        {
            // Add this effect to curr_state 
            auto temp = GroundedCondition(effect.get_predicate(),effect.get_args(),true);
            curr_state.insert(temp);
        }
    }

    return curr_state;

}

void printState(State state)
{
    for(auto const & st:state)
    {
        cout << st;
    }
    cout <<endl;
}

GroundedCondition getGCfromC(Condition cond)
{
    GroundedCondition gc(cond.get_predicate(),cond.get_args(),cond.get_truth());
    return gc;
}

Condition getCfromGC(GroundedCondition cond)
{
    Condition c(cond.get_predicate(),cond.get_arg_values(),cond.get_truth());
    return c;
}

GroundedAction getGAfromA(Action action)
{

    GroundedAction ga(action.get_name(),action.get_args());
    return ga;
}

vector<Action*> GetValidActions(State state, vector<Action*> all_actions)
{

    // cout << "Inside getValidActionsI() function call " << endl;
    // cout << "*************************************************************" << endl;
    // cout << "The state issssssssssss" << endl;
    // printState(state);
    // cout << endl;
    vector<Action*> valid_actions;
    

    for(auto const a : all_actions) // Havent made this a reference as we add a to valid actions
    {
        // cout << "The state issssssssssss" << endl;
        // printState(state);
        // cout << "Action being considered is " << endl;
        // cout << *a;
        int add = 1;
        auto preconditions = a->get_preconditions();
        for(auto const& c:preconditions)
        {
            // cout << "Precondition is " << endl;
            // cout << c;
            auto gc = getGCfromC(c);
            // cout << "AFter converting into grounded condition  " << endl;
            // cout << gc;
            if(state.find(gc) == state.end())
            {
                // cout << "Condition not found in state. This action isn't calid. Break " << endl;
                add = 0;
                break;
            }
        }

        if(add == 1) 
        {
            valid_actions.push_back(a);
            // cout << "Above Action was added " << endl;
        }
        // else cout << "Above action wasn't valid" << endl;

    }

    // cout << "*****************************************************************************" << endl;
    // cout << "Size of valid actions is being returned is  " << valid_actions.size() << endl;
    // cout << "The Valid Actions are " << endl;
    // for(int i = 0; i < valid_actions.size();i++)
    // {
    //     cout << *valid_actions[i] << endl;
    // }
    return valid_actions;

}


double getHeuristic(State curr_state, State goal)
{
    double h = 0;

    for(auto const& gc : goal)
    {
        if(curr_state.find(gc) == goal.end())
        {
            h = h+1;
        }
    }

    return h;
}


// EDL HEuristic

// int getEDLHeuristic(Env* env, State start, State goal,vector<Action*> all_actions)
// {
//     OpenList open_list;
//     ClosedList closed_list;
//     Graph graph;

//     // vector<Action*> all_actions = generateAllPossibleActions(env);

//     // Create start node and add to open list and graph

//     State empty_state;
//     string empty_name;
//     list<string> empty_args;

//     GroundedAction start_ga = GroundedAction(empty_name,empty_args);

//     Node start_node(0.0,0.0,0.0,empty_state,start_ga);

//     open_list.insert(make_pair(0.0,start));
//     graph.insert({start,start_node});
//     auto final_goal_iter = graph.begin();
//     int while_flag = 1;
    

//     while(!open_list.empty() && closed_list.find(goal) == closed_list.end() && while_flag)
//     {

//         cout << "Size of open list is " << open_list.size() << endl;

//         auto curr_open_state = *open_list.begin();
//         auto curr_state = curr_open_state.second;
//         auto curr_f = curr_open_state.first;
//         closed_list.insert(curr_state);
//         open_list.erase(open_list.begin());

//         auto curr_node = graph.find(curr_state);

//         cout << "Print Current state is (State being expanded ) " << endl;
//         printState(curr_state);

//         int flag = 1;
//         for(auto const& gc:goal)
//         {
//             // auto c = getCfromGC(gc);
//             if(curr_state.find(gc) == curr_state.end())
//             {
//                 flag = 0;
//                 break;
//             }
//         }
//         if(flag == 1)
//         {
//             cout << "Current state is the goal state. Astar shoulkd end after this iteration" << endl;
//             cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
//             cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
//             cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
//             cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
//             final_goal_iter = curr_node;
//             while_flag = 0;

//         }

//         vector<Action*> valid_actions = GetValidActions(curr_state,all_actions);

//         for(auto const action : valid_actions) // Should I have & here?
//         {
//             cout << "Action used to generate new state is " << endl;
//             cout << *action;
//             auto new_state = generateSuccessorStateEDL(curr_state,*action);
//             cout << "The new successor state generated is " << endl;
//             printState(new_state);
//             cout << "#########################################################################" << endl;

//             if(closed_list.find(new_state) != closed_list.end())
//             {
                
//                 cout << "Node found in closed list. Go consider the next action . " << endl;
//                 cout << "#########################################################################" << endl;
//                 continue; // Go to next action in valid actions.
//             }


//             // If state is present in graph, check if this is a lower cost path

//             else if(graph.find(new_state) != graph.end())
//             {
//                 cout << "Node found in graph. Check if G value can be decreased " << endl;
//                 cout << "#########################################################################" << endl;
//                 auto iter_new_state = graph.find(new_state);

//                 double newg = curr_node->second.g + 1; //Edge cost =1?
//                 double h = iter_new_state->second.h;
//                 double newf = newg + h;

//                 double oldf = iter_new_state->second.f;

//                 if(newf < oldf)
//                 {
//                     // Update node in both graph and open list
//                     cout << "Better cost path found. Updating g Value" << endl;

//                     open_list.erase(make_pair(iter_new_state->second.f,new_state)); // Find new_state in open list
//                     open_list.insert(make_pair(newf,new_state));

//                     // Update graph 

//                     iter_new_state->second.f = newf;
//                     iter_new_state->second.g = newg;
//                     iter_new_state->second.parent = curr_state; // Current state right??????

//                     iter_new_state->second.ga = getGAfromA(*action) ;

//                 }

            
//             }

//             else
//             {

//                 cout << "Add new node to the graph !!!" << endl;
//                 cout << "#########################################################################" << endl;
//                 double newg = curr_node->second.g +1;
//                 double h = getHeuristic(new_state,goal);
//                 double newf = newg+ h;

//                 open_list.insert(make_pair(newf,new_state));
                
//                 // Creat new node
//                 Node new_node(newf,newg,h,curr_state,getGAfromA(*action));
//                 graph.insert({new_state,new_node});
//             }


//         }

//         cout <<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
//         cout << "Current size of the graph is " << graph.size() << endl;
//         cout <<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;


//     }

//     // Backtrackking to get path

//     cout << "Out of main while loop. Doing backtracking now !" << endl;
//     if(final_goal_iter == graph.begin()) cout << " FInal goal iter = graph.begin(). SOmething went horribly wrong lol" << endl;

//     list<GroundedAction> ga_list;
//     cout << "Size of initialized list of action plan is " << ga_list.size() << endl;

//     auto goal_iter = graph.find(goal);

//     if(goal_iter == graph.end()) cout << "Goal not found in graph. Using final goal iter" << endl;
//     if(goal_iter == graph.end())
//     {
//         goal_iter = final_goal_iter;
//     }
//     auto parent = goal_iter->second.parent;
//     if(goal_iter == graph.end())
//     {
//         cout <<"Goal not found in graph.Wrong print statement" <<endl;
//     }
//     else
//     {
//         cout << "In else conditiooooonnnnnn" << endl;
//         while(!parent.empty())
//         {
//             cout << "In while loop of BAcktrack" << endl;
//             cout << "Parent is " << endl;
//             printState(parent);
//             ga_list.push_back((goal_iter->second.ga));
//             cout << "Action being pushed back in the list is " << endl;
//             cout << goal_iter->second.ga << endl;
//             parent = goal_iter->second.parent;

//             cout << "New parent obtaine is " << endl;
//             if(parent.empty())
//             {
//                 cout << "New parent obtained is empty " << endl;
//             }
//             else printState(parent);

//             goal_iter = graph.find(goal_iter->second.parent);

//         }
//     }


//     cout << "The size of closed list (list of expanded nodes) is " << closed_list.size() << endl;
//     ga_list.pop_back();
//     ga_list.reverse();

//     cout << "Heuristic value being returned is " << ga_list.size() << endl;
//     return ga_list.size();
    

// }





list<GroundedAction> AStarPLanner(Env* env, State start, State goal,vector<Action*> all_actions)
{
    cout << "Entered ASTAR planner function " << endl;
    OpenList open_list;
    ClosedList closed_list;
    Graph graph;

    // vector<Action*> all_actions = generateAllPossibleActions(env);

    // Create start node and add to open list and graph

    State empty_state;
    string empty_name;
    list<string> empty_args;

    GroundedAction start_ga = GroundedAction(empty_name,empty_args);

    Node start_node(0.0,0.0,0.0,empty_state,start_ga);

    open_list.insert(make_pair(0.0,start));
    graph.insert({start,start_node});
    auto final_goal_iter = graph.begin();
    int while_flag = 1;
    

    while(!open_list.empty() && closed_list.find(goal) == closed_list.end() && while_flag)
    {

        // cout << "Size of open list is " << open_list.size() << endl;

        auto curr_open_state = *open_list.begin();
        auto curr_state = curr_open_state.second;
        auto curr_f = curr_open_state.first;
        closed_list.insert(curr_state);
        open_list.erase(open_list.begin());

        auto curr_node = graph.find(curr_state);

        // cout << "Print Current state is (State being expanded ) " << endl;
        // printState(curr_state);

        int flag = 1;
        for(auto const& gc:goal)
        {
            // auto c = getCfromGC(gc);
            if(curr_state.find(gc) == curr_state.end())
            {
                flag = 0;
                break;
            }
        }
        if(flag == 1)
        {
            cout << "Current state is the goal state. Astar should end after this iteration" << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
            //cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
            //cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
            final_goal_iter = curr_node;
            while_flag = 0;

        }

        vector<Action*> valid_actions = GetValidActions(curr_state,all_actions);

        for(auto const action : valid_actions) // Should I have & here?
        {
            // cout << "Action used to generate new state is " << endl;
            // cout << *action;
            auto new_state = generateSuccessorState(curr_state,*action);
            // cout << "The new successor state generated is " << endl;
            // printState(new_state);
            // cout << "#########################################################################" << endl;

            if(closed_list.find(new_state) != closed_list.end())
            {
                
                // cout << "Node found in closed list. Go consider the next action . " << endl;
                // cout << "#########################################################################" << endl;
                continue; // Go to next action in valid actions.
            }


            // If state is present in graph, check if this is a lower cost path

            else if(graph.find(new_state) != graph.end())
            {
                // cout << "Node found in graph. Check if G value can be decreased " << endl;
                // cout << "#########################################################################" << endl;
                auto iter_new_state = graph.find(new_state);

                double newg = curr_node->second.g + 1; //Edge cost =1?
                double h = iter_new_state->second.h;
                double newf = newg + h;

                double oldf = iter_new_state->second.f;

                if(newf < oldf)
                {
                    // Update node in both graph and open list
                    // cout << "Better cost path found. Updating g Value" << endl;

                    open_list.erase(make_pair(iter_new_state->second.f,new_state)); // Find new_state in open list
                    open_list.insert(make_pair(newf,new_state));

                    // Update graph 

                    iter_new_state->second.f = newf;
                    iter_new_state->second.g = newg;
                    iter_new_state->second.parent = curr_state; // Current state right??????

                    iter_new_state->second.ga = getGAfromA(*action) ;

                }

            
            }

            else
            {

                // cout << "Add new node to the graph !!!" << endl;
                // cout << "#########################################################################" << endl;
                double newg = curr_node->second.g +1;
                double h = getHeuristic(new_state,goal); // Changed for EDL
                double newf = newg+ h;

                open_list.insert(make_pair(newf,new_state));
                
                // Creat new node
                Node new_node(newf,newg,h,curr_state,getGAfromA(*action));
                graph.insert({new_state,new_node});
            }


        }

        // cout <<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
        // cout << "Current size of the graph is " << graph.size() << endl;
        // cout <<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;


    }

    // Backtrackking to get path

    cout << "Out of main while loop. Doing backtracking now !" << endl;
    if(final_goal_iter == graph.begin()) cout << " FInal goal iter = graph.begin(). SOmething went horribly wrong lol" << endl;

    list<GroundedAction> ga_list;
    // cout << "Size of initialized list of action plan is " << ga_list.size() << endl;

    auto goal_iter = graph.find(goal);

    if(goal_iter == graph.end()) cout << "Goal not found in graph. Using final goal iter" << endl;
    if(goal_iter == graph.end())
    {
        goal_iter = final_goal_iter;
    }
    auto parent = goal_iter->second.parent;
    if(goal_iter == graph.end())
    {
        cout <<"Goal not found in graph.Wrong print statement" <<endl;
    }
    else
    {
        // cout << "In else conditiooooonnnnnn" << endl;
        while(!parent.empty())
        {
            // cout << "In while loop of BAcktrack" << endl;
            // cout << "Parent is " << endl;
            // printState(parent);
            ga_list.push_back((goal_iter->second.ga));
            // cout << "Action being pushed back in the list is " << endl;
            // cout << goal_iter->second.ga << endl;
            parent = goal_iter->second.parent;

            // cout << "New parent obtaine is " << endl;
            // if(parent.empty())
            // {
            //     cout << "New parent obtained is empty " << endl;
            // }
            // else printState(parent);

            goal_iter = graph.find(goal_iter->second.parent);

        }
    }

    cout << "Number of nodes in the graph is " << graph.size() << endl;
    cout << "The size of closed list (list of expanded nodes) is " << closed_list.size() << endl;
    ga_list.pop_back();
    ga_list.reverse();

    return ga_list;
    

}



// WITHOUT HEURISTIC (DJIKSTRA PLANNER)

list<GroundedAction> DjikstraPLanner(Env* env, State start, State goal,vector<Action*> all_actions)
{
    cout << "Entered djikstra planner function " << endl;
    OpenList open_list;
    ClosedList closed_list;
    Graph graph;

    // vector<Action*> all_actions = generateAllPossibleActions(env);

    // Create start node and add to open list and graph

    State empty_state;
    string empty_name;
    list<string> empty_args;

    GroundedAction start_ga = GroundedAction(empty_name,empty_args);

    Node start_node(0.0,0.0,0.0,empty_state,start_ga);

    open_list.insert(make_pair(0.0,start));
    graph.insert({start,start_node});
    auto final_goal_iter = graph.begin();
    int while_flag = 1;
    

    while(!open_list.empty() && closed_list.find(goal) == closed_list.end() && while_flag)
    {

        // cout << "Size of open list is " << open_list.size() << endl;

        auto curr_open_state = *open_list.begin();
        auto curr_state = curr_open_state.second;
        // auto curr_f = curr_open_state.first;
        closed_list.insert(curr_state);
        open_list.erase(open_list.begin());

        auto curr_node = graph.find(curr_state);

        // cout << "Print Current state is (State being expanded ) " << endl;
        // printState(curr_state);

        int flag = 1;
        for(auto const& gc:goal)
        {
            // auto c = getCfromGC(gc);
            if(curr_state.find(gc) == curr_state.end())
            {
                flag = 0;
                break;
            }
        }
        if(flag == 1)
        {
            cout << "Current state is the goal state. Astar should end after this iteration" << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
            //cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
            //cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << endl;
            final_goal_iter = curr_node;
            while_flag = 0;

        }

        vector<Action*> valid_actions = GetValidActions(curr_state,all_actions);

        for(auto const action : valid_actions) // Should I have & here?
        {
            // cout << "Action used to generate new state is " << endl;
            // cout << *action;
            auto new_state = generateSuccessorState(curr_state,*action);
            // cout << "The new successor state generated is " << endl;
            // printState(new_state);
            // cout << "#########################################################################" << endl;

            if(closed_list.find(new_state) != closed_list.end())
            {
                
                // cout << "Node found in closed list. Go consider the next action . " << endl;
                // cout << "#########################################################################" << endl;
                continue; // Go to next action in valid actions.
            }


            // If state is present in graph, check if this is a lower cost path

            else if(graph.find(new_state) != graph.end())
            {
                // cout << "Node found in graph. Check if G value can be decreased " << endl;
                // cout << "#########################################################################" << endl;
                auto iter_new_state = graph.find(new_state);

                double newg = curr_node->second.g + 1; //Edge cost =1?
                double h = 0;
                double newf = newg + h;

                double oldf = iter_new_state->second.f;

                if(newf < oldf)
                {
                    // Update node in both graph and open list
                    // cout << "Better cost path found. Updating g Value" << endl;

                    open_list.erase(make_pair(iter_new_state->second.f,new_state)); // Find new_state in open list
                    open_list.insert(make_pair(newf,new_state));

                    // Update graph 

                    iter_new_state->second.f = newf;
                    iter_new_state->second.g = newg;
                    iter_new_state->second.parent = curr_state; // Current state right??????

                    iter_new_state->second.ga = getGAfromA(*action) ;

                }

            
            }

            else
            {

                // cout << "Add new node to the graph !!!" << endl;
                // cout << "#########################################################################" << endl;
                double newg = curr_node->second.g +1;
                double h = 0; // Changed for EDL
                double newf = newg+ h;

                // cout << "Heuristic value in Djikstra is " << h << "---------------------------------------------------------------------" << endl;

                open_list.insert(make_pair(newf,new_state));
                
                // Creat new node
                Node new_node(newf,newg,h,curr_state,getGAfromA(*action));
                graph.insert({new_state,new_node});
            }


        }

        // cout <<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
        // cout << "Current size of the graph is " << graph.size() << endl;
        // cout <<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;


    }

    // Backtrackking to get path

    // cout << "Out of main while loop. Doing backtracking now !" << endl;
    if(final_goal_iter == graph.begin()) cout << " FInal goal iter = graph.begin(). SOmething went horribly wrong lol" << endl;

    list<GroundedAction> ga_list;
    // cout << "Size of initialized list of action plan is " << ga_list.size() << endl;

    auto goal_iter = graph.find(goal);

    // if(goal_iter == graph.end()) cout << "Goal not found in graph. Using final goal iter" << endl;
    if(goal_iter == graph.end())
    {
        goal_iter = final_goal_iter;
    }
    auto parent = goal_iter->second.parent;
    if(goal_iter == graph.end())
    {
        cout <<"Goal not found in graph.Wrong print statement" <<endl;
    }
    else
    {
        // cout << "In else conditiooooonnnnnn" << endl;
        while(!parent.empty())
        {
            // cout << "In while loop of BAcktrack" << endl;
            // cout << "Parent is " << endl;
            // printState(parent);
            ga_list.push_back((goal_iter->second.ga));
            // cout << "Action being pushed back in the list is " << endl;
            // cout << goal_iter->second.ga << endl;
            parent = goal_iter->second.parent;

            // cout << "New parent obtaine is " << endl;
            // if(parent.empty())
            // {
            //     cout << "New parent obtained is empty " << endl;
            // }
            // else printState(parent);

            goal_iter = graph.find(goal_iter->second.parent);

        }
    }

    cout << "Size of graph is : " << graph.size() << endl;
    cout << "The size of closed list (list of expanded nodes) is " << closed_list.size() << endl;
    ga_list.pop_back();
    ga_list.reverse();

    return ga_list;
    

}










// PLANNER FUNCTION

list<GroundedAction> planner(Env* env)
{
    // this is where you insert your planner
    cout << "Entered Planner function call" << endl;
    cout << "************************************************************" << endl;
    list<GroundedAction> actions;

    std::vector<Action*> all_actions = generateAllPossibleActions(env);
    cout << "Finished Generating the vector of all possible actions " << endl;
    cout << "************************************************************" << endl;
    cout << "Number of possible actions generated is " << all_actions.size() << endl;
    cout << "************************************************************" << endl;

    auto start = env->get_start_state();
    auto goal = env->get_goal_state();
    actions = AStarPLanner(env,start,goal,all_actions);
    // actions = DjikstraPLanner(env,start,goal,all_actions);

    return actions;
}


int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("Blocks.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    cout << "************************************************************" << endl;
    cout << "FInished creating envoironment. Call the planner function" << endl;
    cout << "************************************************************" << endl;

    auto start = std::chrono::high_resolution_clock::now();

    list<GroundedAction> actions = planner(env);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> seconds = end - start;
    std::cout << "Time taken for Planning is " << seconds.count() << std::endl;

    cout << "Length of Plan obtained is : " << actions.size() << "actions " << endl;


    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}