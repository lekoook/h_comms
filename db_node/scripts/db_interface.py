import abc

class FormalDatabaseInterface(metaclass=abc.ABCMeta):
    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass,"generate_mit") and
            callable(subclass.generate_mit) and
            hasattr(subclass,"write_to_db") and
            callable(subclass.write_to_db)
        )

