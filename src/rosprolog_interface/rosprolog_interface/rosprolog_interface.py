import rospy
from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish, PrologQueryRequest, PrologNextSolutionRequest, PrologFinishRequest
from typing import Iterator, Dict, List, Optional
import pycram.external_interfaces.knowrob_designator_client as kdc

class PrologQueryWrapper:
    def __init__(self, query_str: str, simple_query_srv, next_solution_srv, finish_srv, iterative=True):
        """
        Wraps around ROS Prolog services for a convenient Python interface.
        :param iterative: if False, all solutions will be calculated by rosprolog during the first service call
        """
        self._simple_query_srv = simple_query_srv
        self._next_solution_srv = next_solution_srv
        self._finish_query_srv = finish_srv
        self._finished = False
        self._query_id = None

        # Request to initiate the query
        try:
            request = PrologQueryRequest()
            request.query = query_str
            request.mode = 1 if iterative else 0
            response = self._simple_query_srv(request)
            if not response.ok:
                raise Exception(f'Prolog query failed: {response.message}')
            #self._query_id = response.id
        except rospy.ServiceException as e:
            raise Exception(f"Service call failed: {e}")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.finish()

    def solutions(self) -> Iterator[Dict]:
        """Retrieve solutions from the query one at a time."""
        try:
            while not self._finished:
                request = PrologNextSolutionRequest(id=self._query_id)
                response = self._next_solution_srv(request)
                if response.status == response.OK:
                    yield self._json_to_dict(response.solution)
                elif response.status == response.NO_SOLUTION:
                    break
                else:
                    raise Exception(f"Unknown query status {response.status}")
        finally:
            self.finish()

    def finish(self):
        """Finish the Prolog query and release resources."""
        if not self._finished:
            try:
                request = PrologFinishRequest(id=self._query_id)
                self._finish_query_srv(request)
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to finish query: {e}")
            finally:
                self._finished = True

    def _json_to_dict(self, json_text):
        import json
        return json.loads(json_text)

class Prolog:
    def __init__(self, namespace='rosprolog'):
        """Initialize Prolog service clients."""
        rospy.loginfo("[KnowRob] initialize client...")
        rospy.wait_for_service(f'{namespace}/query')
        rospy.wait_for_service(f'{namespace}/next_solution')
        rospy.wait_for_service(f'{namespace}/finish')

        self._simple_query_srv = rospy.ServiceProxy(f'{namespace}/query', PrologQuery)
        self._next_solution_srv = rospy.ServiceProxy(f'{namespace}/next_solution', PrologNextSolution)
        self._finish_query_srv = rospy.ServiceProxy(f'{namespace}/finish', PrologFinish)
        self.all_solutions(f"init_gpsr_2024.") # intit semantic map things
        rospy.loginfo("[KnowRob]  done.")

    def query(self, query_str):
        """Execute a Prolog query, yielding solutions iteratively."""
        return PrologQueryWrapper(query_str, simple_query_srv=self._simple_query_srv,
                                  next_solution_srv=self._next_solution_srv, finish_srv=self._finish_query_srv)



    def once(self, query_str: str) -> Optional[Dict]:
        """Retrieve one solution from the query and finish immediately."""
        # differenciate between fallschool designator queries and normal prolog
        if "type=" in query_str:
            return kdc.send_simple_query(query_str)
        with PrologQueryWrapper(query_str, self._simple_query_srv, self._next_solution_srv, self._finish_query_srv) as query:
            try:
                solution = next(query.solutions())
                if 0 == len(solution):
                    return True
                else:
                    return solution
            except StopIteration:
                return False

    def all_solutions(self, query_str: str) -> List[Dict]:
        """Retrieve all solutions at once."""
        # differenciate between fallschool designator queries and normal prolog
        if "type=" in query_str:
            return kdc.send_simple_query(query_str)
        with PrologQueryWrapper(query_str, self._simple_query_srv, self._next_solution_srv, self._finish_query_srv, iterative=False) as query:
            solution = list(query.solutions())
            if 0 == len(solution):
                return False
            elif (1 == len(solution)) and (0 == len(solution[0])):
                return True
            else:
                return solution

# Entity Prototype
# Examples:
# _toEntityQuery('entity (an Object (type=Milk, storagePlace =  ?storagePlace))')
# 'EntityClass = Object, Determiner = an, type(EntityName, Type), Type = Milk, storagePlace(EntityName, StoragePlace).'
class Entity:
    def __init__(self, **kwargs):
        self.__dict__ = kwargs
    def __repr__(self):
        def _lowerInitial(s):
            return s[0].lower() + s[1:] if s else ''
        retq = ", ".join(["%s: %s" % (_lowerInitial(k), v) for k, v in self.__dict__.items() if k not in ["Determiner", "EntityClass"]])
        retq = f"({retq})"
        if "EntityClass" in self.__dict__:
            retq = self.__dict__["EntityClass"] + retq
        if "Determiner" in self.__dict__:
            retq = self.__dict__["Determiner"] + " " + retq
        if "EntityClass" not in self.__dict__:
            return f"entity{retq}"
        return f"entity({retq})"

def _toEntityQuery(query_str):
    def _upperInitial(s):
        return s[0].upper() + s[1:] if s else ''
    query_str = query_str.strip()
    if (not query_str.startswith("entity")):
        raise ValueError("Query string must start with 'entity")
    query_str = query_str[len("entity"):].strip()
    if (not query_str.startswith("(")) or (not query_str.endswith(")")):
        raise ValueError("Entity must be enclosed in brackets.")
    query_str = query_str[1:-1].strip()
    if (not query_str.startswith("an ")) and (not query_str.startswith("a ")) and (not query_str.startswith("the ")):
        raise ValueError("Query string must have an article in the beginning")
    determiner, query_str = query_str.split(" ", 1)
    query_str = query_str.strip()
    aux = query_str.split("(", 1)
    if 0 == len(aux):
        raise ValueError("Entity Class not found.")
    entityClass, query_str = aux
    entityClass = entityClass.strip()
    query_str = query_str.strip()
    if not query_str.endswith(")"):
        raise ValueError("Entity Class does not close its bracket.")
    query_str=query_str[:-1]
    query_str = query_str.strip()
    attributes = query_str.split(",")
    retq = f"EntityClass = {entityClass}, Determiner = {determiner}"
    for a in attributes:
        aux = a.split("=", 1)
        if 2 != len(aux):
            raise ValueError("Attribute does not have a value.")
        key, value = aux
        key = key.strip()
        value = value.strip()
        if value.startswith("?"):
            value = _upperInitial(value[1:])
            retq += f", {key}(EntityName, {value})"
        else:
            var = _upperInitial(key)
            retq += f", {key}(EntityName, {var}), {var} = {value}"
    return retq + "."

def entity_query_once(knowrob, prolog_query):
    answer = knowrob.once(prolog_query)
    if answer is False:
        return None
    if answer is True:
        return Entity()
    return Entity(**answer)


def entity_query_all(knowrob, prolog_query):
    answer = knowrob.all_solutions(prolog_query)
    if answer is False:
        return None
    if answer is True:
        return [Entity()]
    return [Entity(**x) for x in answer]
