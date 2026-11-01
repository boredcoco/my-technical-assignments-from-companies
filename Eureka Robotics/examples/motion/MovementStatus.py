from enum import Enum
from typing import Union


class MovementStatus(str, Enum):
    RUNNING   = "running"
    REACHED   = "reached"
    BUMPED    = "bumped"
    CANCELLED = "cancelled"
    PREEMPTED = "preempted"
    UNKNOWN   = "unknown"
    DID_NOT_REACH = "did_not_reach"

    @classmethod
    def parse(cls, status: Union[str, "MovementStatus", None]) -> "MovementStatus":
        if status is None:
            return cls.UNKNOWN
        if isinstance(status, cls):
            return status
        if not isinstance(status, str):
            raise ValueError(f"Movement status must be a str, got {type(status).__name__}")
        key = status.strip().lower()
        if not key:
            return cls.UNKNOWN
        try:
            return cls(key)
        except ValueError:
            valid = ", ".join(m.value for m in cls if m is not cls.UNKNOWN)
            raise ValueError(
                f"Invalid movement status {status!r}; expected one of: {valid}"
            ) from None

    @property
    def is_terminal(self) -> bool:
        return self in {
            MovementStatus.REACHED,
            MovementStatus.BUMPED,
            MovementStatus.CANCELLED,
            MovementStatus.PREEMPTED,
        }

    @property
    def is_success(self) -> bool:
        return self is MovementStatus.REACHED
