"""UI-independent semantic scene state with bounded undo and redo."""

from copy import deepcopy


class SemanticScene:
    def __init__(self, semantic_map, history_limit=100):
        if history_limit <= 0:
            raise ValueError("history_limit must be positive")
        self.semantic_map = deepcopy(semantic_map)
        self.history_limit = int(history_limit)
        self.selected_feature_id = None
        self.dirty = False
        self._undo_stack = []
        self._redo_stack = []

    @property
    def features(self):
        return self.semantic_map.features

    @property
    def can_undo(self):
        return bool(self._undo_stack)

    @property
    def can_redo(self):
        return bool(self._redo_stack)

    def get(self, feature_id):
        return next(
            (feature for feature in self.features if feature.id == feature_id),
            None,
        )

    def add(self, feature):
        if self.get(feature.id) is not None:
            raise ValueError(f"duplicate feature id: {feature.id}")
        self._record_state()
        self.features.append(deepcopy(feature))
        self.selected_feature_id = feature.id
        self.dirty = True

    def replace(self, feature):
        self.replace_by_id(feature.id, feature)

    def replace_by_id(self, current_id, feature):
        existing = self.get(feature.id)
        if existing is not None and feature.id != current_id:
            raise ValueError(f"duplicate feature id: {feature.id}")
        for index, current in enumerate(self.features):
            if current.id == current_id:
                self._record_state()
                self.features[index] = deepcopy(feature)
                self.selected_feature_id = feature.id
                self.dirty = True
                return
        raise KeyError(current_id)

    def remove(self, feature_id):
        for index, feature in enumerate(self.features):
            if feature.id == feature_id:
                self._record_state()
                removed = self.features.pop(index)
                if self.selected_feature_id == feature_id:
                    self.selected_feature_id = None
                self.dirty = True
                return removed
        raise KeyError(feature_id)

    def undo(self):
        if not self._undo_stack:
            return False
        self._redo_stack.append(self._snapshot())
        self._restore(self._undo_stack.pop())
        self.dirty = True
        return True

    def redo(self):
        if not self._redo_stack:
            return False
        self._undo_stack.append(self._snapshot())
        self._restore(self._redo_stack.pop())
        self.dirty = True
        return True

    def mark_saved(self):
        self.dirty = False

    def _record_state(self):
        self._undo_stack.append(self._snapshot())
        if len(self._undo_stack) > self.history_limit:
            self._undo_stack.pop(0)
        self._redo_stack.clear()

    def _snapshot(self):
        return deepcopy((self.semantic_map, self.selected_feature_id))

    def _restore(self, snapshot):
        self.semantic_map, self.selected_feature_id = deepcopy(snapshot)
