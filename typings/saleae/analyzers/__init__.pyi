from typing import Any, Dict, Iterable, List, Optional, Union

class SaleaeTimeDelta:
    def __float__(self) -> float: ...

class SaleaeTime:
    def __sub__(self, other: "SaleaeTime") -> SaleaeTimeDelta: ...
    def __add__(self, other: Any) -> "SaleaeTime": ...

class AnalyzerFrame:
    type: str
    start_time: SaleaeTime
    end_time: SaleaeTime
    data: Dict[str, Any]
    def __init__(self, type: str, start_time: SaleaeTime, end_time: SaleaeTime,
                 data: Optional[Dict[str, Any]] = ...) -> None: ...

class HighLevelAnalyzer:
    result_types: Dict[str, Dict[str, str]]
    def decode(self, frame: AnalyzerFrame) -> Union[None, AnalyzerFrame, List[AnalyzerFrame]]: ...

class StringSetting:
    def __init__(self, label: str = ..., **kwargs: Any) -> None: ...

class NumberSetting:
    def __init__(self, label: str = ..., min_value: float = ..., max_value: float = ..., **kwargs: Any) -> None: ...

class ChoicesSetting:
    def __init__(self, choices: Iterable[str], label: str = ..., **kwargs: Any) -> None: ...