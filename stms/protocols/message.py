"""
FIPA-ACL (Foundation for Intelligent Physical Agents - Agent Communication Language)
message model and in-memory broker for the Smart Traffic Management System.

Owner: Member 5 (Coordinator & Protocols)

Interaction protocols used in this project
------------------------------------------
1. Emergency preemption  (fipa-request style)
       Ambulance --PREEMPT_OVERRIDE--> Intersection
       Intersection --AGREE--> Ambulance
       Ambulance --CANCEL--> Intersection          (after it has crossed)
2. Green-wave platoon hand-off (peer-to-peer INFORM)
       Intersection --INFORM(PLATOON_INCOMING)--> downstream Intersection
3. Incident management / advisories (one-to-many BROADCAST)
       Coordinator --BROADCAST(INCIDENT_ALERT)--> all agents
       Coordinator --INFORM(CONGESTION_WARNING)--> all agents
       Coordinator --INFORM(PARAMETER_UPDATE)--> Intersections  (hill-climbing tuner)
"""

from collections import Counter
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional
import itertools


class Performative(str, Enum):
    """FIPA-ACL communicative acts (plus one domain-specific act, PREEMPT_OVERRIDE)."""
    INFORM = "INFORM"
    REQUEST = "REQUEST"
    PROPOSE = "PROPOSE"
    AGREE = "AGREE"
    REFUSE = "REFUSE"
    ACCEPT_PROPOSAL = "ACCEPT_PROPOSAL"
    REJECT_PROPOSAL = "REJECT_PROPOSAL"
    CANCEL = "CANCEL"
    QUERY_IF = "QUERY_IF"
    BROADCAST = "BROADCAST"
    PREEMPT_OVERRIDE = "PREEMPT_OVERRIDE"   # domain extension of REQUEST with top priority


_conv_counter = itertools.count(1)


def _new_conversation_id() -> str:
    return f"C{next(_conv_counter):06d}"


@dataclass
class ACLMessage:
    """Standard FIPA-ACL message envelope."""
    sender_id: str
    receiver_id: str
    performative: Performative
    content: Dict[str, Any]
    ontology: str = "traffic-management"
    protocol: str = "fipa-acl"
    conversation_id: str = field(default_factory=_new_conversation_id)
    reply_with: Optional[str] = None
    in_reply_to: Optional[str] = None
    timestamp: float = 0.0          # simulation time (seconds), stamped by the broker

    def create_reply(self, performative: Performative, content: Dict[str, Any]) -> "ACLMessage":
        """Creates a reply in the same conversation thread."""
        return ACLMessage(
            sender_id=self.receiver_id,
            receiver_id=self.sender_id,
            performative=performative,
            content=content,
            ontology=self.ontology,
            protocol=self.protocol,
            conversation_id=self.conversation_id,
            in_reply_to=self.reply_with or self.conversation_id,
        )

    def short(self) -> str:
        return f"t={self.timestamp:5.0f}s  {self.sender_id:>13} --{self.performative.value}--> {self.receiver_id:<13} {self.content}"

    def __repr__(self) -> str:
        return f"ACLMessage({self.performative.value} {self.sender_id}->{self.receiver_id} {self.content})"


class MessageBroker:
    """
    In-memory message transport. Supports point-to-point, BROADCAST and TOPIC:<name> delivery.
    Keeps a full history so tests and the report can audit every interaction.
    """

    def __init__(self, keep_history: bool = True):
        self._mailboxes: Dict[str, List[ACLMessage]] = {}
        self._subscriptions: Dict[str, List[str]] = {}
        self._history: List[ACLMessage] = []
        self.keep_history = keep_history
        self.sim_time: float = 0.0
        self.counts: Counter = Counter()
        self.total_messages_dispatched: int = 0

    # ---- registration -------------------------------------------------
    def register_agent(self, agent_id: str):
        self._mailboxes.setdefault(agent_id, [])

    def unregister_agent(self, agent_id: str):
        self._mailboxes.pop(agent_id, None)
        for subs in self._subscriptions.values():
            if agent_id in subs:
                subs.remove(agent_id)

    def subscribe(self, agent_id: str, topic: str):
        self.register_agent(agent_id)
        subs = self._subscriptions.setdefault(topic, [])
        if agent_id not in subs:
            subs.append(agent_id)

    def is_registered(self, agent_id: str) -> bool:
        return agent_id in self._mailboxes

    # ---- transport ----------------------------------------------------
    def send(self, message: ACLMessage) -> bool:
        """Delivers a message. Returns False if a point-to-point receiver does not exist."""
        message.timestamp = self.sim_time
        self.total_messages_dispatched += 1
        self.counts[message.performative.value] += 1
        if self.keep_history:
            self._history.append(message)

        if message.receiver_id == "BROADCAST":
            for aid, mbox in self._mailboxes.items():
                if aid != message.sender_id:
                    mbox.append(message)
            return True
        if message.receiver_id.startswith("TOPIC:"):
            topic = message.receiver_id.split(":", 1)[1]
            for aid in self._subscriptions.get(topic, []):
                if aid != message.sender_id and aid in self._mailboxes:
                    self._mailboxes[aid].append(message)
            return True
        if message.receiver_id in self._mailboxes:
            self._mailboxes[message.receiver_id].append(message)
            return True
        return False

    def receive(self, agent_id: str) -> List[ACLMessage]:
        box = self._mailboxes.get(agent_id)
        if not box:
            return []
        pending = list(box)
        box.clear()
        return pending

    def peek(self, agent_id: str) -> List[ACLMessage]:
        return list(self._mailboxes.get(agent_id, []))

    # ---- auditing -----------------------------------------------------
    def history(self, performative: Optional[Performative] = None,
                ontology: Optional[str] = None) -> List[ACLMessage]:
        out = self._history
        if performative is not None:
            out = [m for m in out if m.performative == performative]
        if ontology is not None:
            out = [m for m in out if m.ontology == ontology]
        return list(out)

    def clear(self):
        for mbox in self._mailboxes.values():
            mbox.clear()
        self._history.clear()
        self.counts.clear()
        self.total_messages_dispatched = 0
