from pynput import keyboard

class ToggleManager:
    def __init__(self, keys):
        self._keys = keys
        # Ensure all keys are initialized in the toggles dictionary.
        for key in keys:
            if key not in toggles:
                toggles[key] = False
        self._prev_toggles = self.get_current_toggles()

    def get_current_toggles(self):
        # Return a list of the keys that are currently toggled.
        return [key for key in self._keys if toggles[key]]

    def get_active(self):
        current_toggles = self.get_current_toggles()
        if len(current_toggles) < 2: # If not multiple toggles are active, return the only one.
            self._prev_toggles = current_toggles
            return next(iter(current_toggles), None)
        
        # Get the newly toggled key that wasn't in the previous state.
        new_toggles = [key for key in current_toggles if key not in self._prev_toggles]
        new_toggle = next(iter(new_toggles), None) # Pick the first new toggle.

        # Set all keys to False except the new toggle
        for key in self._keys:
            toggles[key] = key == new_toggle

        # Update the previous toggles for the next check.
        self._prev_toggles = self.get_current_toggles()
        return new_toggle

# Global sets/dictionaries to track state.
pressed_keys = set()
just_pressed_keys = set()
just_released_keys = set()
toggles = {}

def is_pressed(key):
    """Checks if a key is currently held down."""
    key_repr = key if isinstance(key, str) else key.char if hasattr(key, 'char') else str(key)
    return key_repr in pressed_keys

def is_toggled(key):
    """Returns the toggle state of a key. If the key is not registered, it initializes it to False."""
    key_repr = key if isinstance(key, str) else key.char if hasattr(key, 'char') else str(key)
    if key_repr not in toggles: # Register the key as a toggle if not already present
        toggles[key_repr] = False
    return toggles.get(key_repr, False)

def rising_edge(key):
    """Returns True on the first press of a key until it's released and pressed again."""
    key_repr = key if isinstance(key, str) else key.char if hasattr(key, 'char') else str(key)
    if key_repr in just_pressed_keys:
        just_pressed_keys.remove(key_repr)
        return True
    return False

def falling_edge(key):
    """Returns True on the first release of a key until it's pressed and released again."""
    key_repr = key if isinstance(key, str) else key.char if hasattr(key, 'char') else str(key)
    if key_repr in just_released_keys:
        just_released_keys.remove(key_repr)
        return True
    return False

def _on_press(key):
    key_repr = key if isinstance(key, str) else key.char if hasattr(key, 'char') else str(key)
    # Only mark as just pressed if it wasn't already noted as down.
    if key_repr not in pressed_keys:
        just_pressed_keys.add(key_repr)
    pressed_keys.add(key_repr)
    # Toggle state if applicable.
    if key_repr in toggles:
        toggles[key_repr] = not toggles[key_repr]

def _on_release(key):
    key_repr = key if isinstance(key, str) else key.char if hasattr(key, 'char') else str(key)
    pressed_keys.discard(key_repr)
    # Mark the key as just released for falling edge detection.
    just_released_keys.add(key_repr)
    # Optionally, you might also remove from just_pressed_keys if desired.
    # just_pressed_keys.discard(key_repr)

keyboard.Listener(on_press=_on_press, on_release=_on_release).start()

# Example usage
if __name__ == '__main__':
    import time

    radio_buttons = ToggleManager(['1', '2', '3'])
    while True:
        if is_pressed('w'):
            print("W key is held down")
        if is_toggled('t'):
            print("T key toggled")
        if rising_edge('q'):
            print("Q key was pressed")
        if falling_edge('q'):
            print("Q key was released")
        active_key = radio_buttons.get_active()
        if active_key:
            print(f"Active key: {active_key}")
        # Optionally print all pressed keys
        if pressed_keys:
            print(f"Pressed keys: {', '.join(pressed_keys)}")
        time.sleep(0.1)
