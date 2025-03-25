import yaml
import random
import os
import rospkg

class Receptionist:
    def __init__(self, knowledge_file='receptionist_knowledge.yaml'):
        rospack = rospkg.RosPack()
        package = rospack.get_path('config_files')
        # package = os.path.dirname(__file__)
        self.file_path = os.path.join(package, knowledge_file)
        self.places = self.__load_places_from_yaml()
        self.people = {}
        self.active_guest = None
        self.active_seat = None
        self.guest_assigned = 0
        self.unknown_count = 0

    def __load_places_from_yaml(self):
        if not os.path.exists(self.file_path):
            print(f"El archivo {self.file_path} no existe. Creando estructura vacía.")
            return {}
        try:
            with open(self.file_path, 'r') as file:
                data = yaml.safe_load(file) or {}
                return data.get("places", {})
        except yaml.YAMLError as e:
            print(f"Error al leer el archivo YAML: {e}")
            return {}

    def add_guest(self, name):
        if not name:
            print("Intento de agregar un invitado sin nombre.")
            return
        name = name.lower()
        self.active_guest = self.add_new_person(name)
        self.active_seat = self.get_any_available_seat()
    
    def add_guest_drink(self, drink):
        drink = drink.lower()
        if self.active_guest and self.active_guest in self.people:
            self.people[self.active_guest]["drink"] = drink

    def add_guest_interest(self, interest):
        interest = interest.lower()
        if self.active_guest and self.active_guest in self.people:
            self.people[self.active_guest]["interest"] = interest

    def add_guest_description(self, description):
        description = description.lower()
        if self.active_guest and self.active_guest in self.people:
            self.people[self.active_guest]["description"] = description

    def add_new_person(self, name):
        guest_num = f"Guest_{len(self.people)}"
        self.people[guest_num] = {
            "name": name, 
            "drink": None, 
            "interest": None, 
            "location": None, 
            "description": None}
        return guest_num
    
    def get_active_guest_name(self):
        return self.people.get(self.active_guest, {}).get("name")

    def get_any_available_seat(self):
        seats = self.__get_available_seats()
        return random.choice(seats) if seats else None

    def __get_available_seats(self):
        return [place for place, info in self.places.items() if not info.get("occupied")]

    def get_guest_by_name(self, guest_name):
        return next((guest for guest, info in self.people.items() if info.get("name") == guest_name), None)
    
    def get_active_guest_drink(self):
        return self.people.get(self.active_guest, {}).get("drink")
    
    def get_active_seat(self):
        return self.active_seat
    
    def get_guest_description(self, guest_num):
        return self.people.get(guest_num, {}).get("description")
    
    def get_guest_interest(self, guest_num):
        return self.people.get(guest_num, {}).get("interest")

    def seat_confirmation(self, detected_person=None):
        if detected_person:
            detected_person = detected_person.lower()
            guest_num = self.get_guest_by_name(detected_person)
            if guest_num:
                last_seat = self.people[guest_num].get("location")
                if last_seat and last_seat in self.places:
                    self.places[last_seat]["occupied"] = None
                self.places[self.active_seat]["occupied"] = guest_num
                self.people[guest_num]["location"] = self.active_seat
                self.active_seat = self.get_any_available_seat()
            else:
                someone_id = f"someone_{len([p for p in self.places.values() if 'someone' in str(p.get('occupied', ''))])}"
                self.places[self.active_seat]["occupied"] = someone_id
                self.active_seat = self.get_any_available_seat()
        else:
            if self.active_guest and self.active_seat:
                self.places[self.active_seat]["occupied"] = self.active_guest
                self.people[self.active_guest]["location"] = self.active_seat
                self.active_seat = None
                self.active_guest = None
                self.guest_assigned += 1

# --- Test ---
if __name__ == '__main__':
    party = Receptionist()

    # 1. Llega Rubén
    print("\n--- Llega Rubén ---")
    party.add_guest("Ruben")
    print(f"Rubén asignado como {party.active_guest}")

    party.add_guest_drink("Coke")
    print(f"{party.get_active_guest_name()} prefiere {party.get_active_guest_drink()}")

    party.add_guest_interest("Movies")
    print(f"{party.get_active_guest_name()} tiene interés en {party.people[party.active_guest]['interest']}")

    party.add_guest_description("Mexicano y guapo")
    print(f"{party.get_active_guest_name()} descripción: {party.people[party.active_guest]['description']}")

    seat_assigned = party.active_seat
    party.seat_confirmation(None)  # No hay nadie en el asiento
    print(f"{party.get_active_guest_name()} se sienta en {party.people[party.active_guest]['location']}")

    # 2. Llega John
    print("\n--- Llega John ---")
    party.add_guest("John")
    print(f"John asignado como {party.active_guest}")

    party.add_guest_drink("Water")
    print(f"{party.get_active_guest_name()} prefiere {party.get_active_guest_drink()}")

    party.add_guest_interest("Listening to music")
    print(f"{party.get_active_guest_name()} tiene interés en {party.people[party.active_guest]['interest']}")

    party.add_guest_description("Afroamericano y alto")
    print(f"{party.get_active_guest_name()} descripción: {party.people[party.active_guest]['description']}")

    seat_assigned = party.active_seat
    print(f"{party.get_active_guest_name()} intentará sentarse en {seat_assigned}")

    # Simulación: John intenta sentarse pero encuentra que Rubén se movió ahí
    detected_person = "Ruben"
    party.seat_confirmation(detected_person)

    print(f"{detected_person} fue detectado en el asiento, reasignando a {party.active_guest}...")
    print(f"{party.get_active_guest_name()} fue reasignado de asiento a {party.active_seat}")
    print(f"No se encuentra a ninguna persona en {party.active_seat}")
    detected_person = None
    party.seat_confirmation(detected_person)
    print(f"{party.get_active_guest_name()} ahora está en {party.people[party.active_guest]['location']}")

    # Mostrar estado final de los asientos
    print("\n--- Estado Final de Asientos ---")
    for seat, info in party.places.items():
        print(f"{seat}: {info.get('occupied', 'Libre')}")

