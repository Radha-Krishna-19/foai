"""
Base agent: the Sense -> Reason -> Act cycle shared by every agent in the system.
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Dict, List, Optional
from src.protocols.message import ACLMessage, MessageBroker, Performative


class AgentType(str, Enum):
    INTERSECTION = "INTERSECTION"
    VEHICLE = "VEHICLE"
    EMERGENCY = "EMERGENCY"
    BUS = "BUS"
    COORDINATOR = "COORDINATOR"


class BaseAgent(ABC):
    """
    Abstract agent (Russell & Norvig agent function  f : P* -> A).

    One simulation tick calls step(), which runs exactly once:
        1. receive_messages()   - pull FIPA-ACL messages from the broker
        2. perceive(env)        - update internal state from sensors + inbox
        3. decision = reason()  - deliberate (search / pressure / utility)
        4. act(env)             - apply self.decision to the environment
    """

    def __init__(self, agent_id: str, agent_type: AgentType, broker: MessageBroker):
        self.agent_id = agent_id
        self.agent_type = agent_type
        self.broker = broker
        self.inbox: List[ACLMessage] = []
        self.active: bool = True
        self.decision: Any = None
        self.sim_time: float = 0.0
        self.broker.register_agent(self.agent_id)

    # ---- communication -----------------------------------------------
    def receive_messages(self) -> List[ACLMessage]:
        new_msgs = self.broker.receive(self.agent_id)
        self.inbox.extend(new_msgs)
        return new_msgs

    def send_message(self, receiver_id: str, performative: Performative, content: Dict[str, Any],
                     ontology: str = "traffic-management", protocol: str = "fipa-acl",
                     in_reply_to: Optional[str] = None,
                     conversation_id: Optional[str] = None) -> ACLMessage:
        msg = ACLMessage(sender_id=self.agent_id, receiver_id=receiver_id, performative=performative,
                         content=content, ontology=ontology, protocol=protocol, in_reply_to=in_reply_to)
        if conversation_id:
            msg.conversation_id = conversation_id
        self.broker.send(msg)
        return msg

    def broadcast(self, performative: Performative, content: Dict[str, Any],
                  ontology: str = "traffic-management") -> ACLMessage:
        return self.send_message("BROADCAST", performative, content, ontology=ontology)

    # ---- agent program ------------------------------------------------
    @abstractmethod
    def perceive(self, env: Dict[str, Any]):
        ...

    @abstractmethod
    def reason(self) -> Any:
        ...

    @abstractmethod
    def act(self, env: Dict[str, Any]):
        ...

    def step(self, env: Dict[str, Any]):
        if not self.active:
            return
        self.sim_time = env.get("time", self.sim_time)
        self.receive_messages()
        self.perceive(env)
        self.decision = self.reason()
        self.act(env)
