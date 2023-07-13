#[cfg(test)]
macro_rules! serialization_test {
    (type = $type:ty, $({ $name:ident, $original:expr, le = $le:expr, be = $be:expr }),+) => {
        $(mod $name {
            use super::*;
            use speedy::{Readable, Writable, Endianness};

            #[test]
            fn serialize_little_endian() {
                let original: $type = $original;
                let serialized = original.write_to_vec_with_ctx(Endianness::LittleEndian)
                    .expect(&format!("serialize failed from {original:?}"));
                assert_eq!(serialized, $le);
            }

            #[test]
            fn serialize_big_endian() {
                let original: $type = $original;
                let serialized = original.write_to_vec_with_ctx(Endianness::BigEndian).unwrap();
                assert_eq!(serialized, $be);
            }

            #[test]
            fn serialize_deserialize_little_endian() {
                let original: $type = $original;

                let serialized = original.write_to_vec_with_ctx(Endianness::LittleEndian).unwrap();
                let deserialized: $type =
                    Readable::read_from_buffer_with_ctx(Endianness::LittleEndian, &serialized)
                    .expect(&format!("deserialize failed from {:?} original={:?}",
                                    serialized, original));

                assert_eq!(original, deserialized);
            }

            #[test]
            fn serialize_deserialize_big_endian() {
                let original: $type = $original;

                let serialized = original.write_to_vec_with_ctx(Endianness::BigEndian).unwrap();
                let deserialized: $type =
                    Readable::read_from_buffer_with_ctx(Endianness::BigEndian, &serialized)
                    .unwrap();

                assert_eq!(original, deserialized);
            }
        })+
    }
}
