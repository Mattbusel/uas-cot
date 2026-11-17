//! MAVLink XML dialect definitions: messages, enums and the MAVLink 2 wire
//! layout (field order, offsets, `CRC_EXTRA`) computed from them.

use std::collections::BTreeMap;
use std::fmt;
use std::path::Path;

/// A field's primitive type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldType {
    /// `char`
    Char,
    /// `uint8_t` (and `uint8_t_mavlink_version`)
    U8,
    /// `int8_t`
    I8,
    /// `uint16_t`
    U16,
    /// `int16_t`
    I16,
    /// `uint32_t`
    U32,
    /// `int32_t`
    I32,
    /// `float`
    F32,
    /// `uint64_t`
    U64,
    /// `int64_t`
    I64,
    /// `double`
    F64,
}

impl FieldType {
    /// Parses a MAVLink XML primitive type name (without the array suffix).
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "char" => FieldType::Char,
            "uint8_t" | "uint8_t_mavlink_version" => FieldType::U8,
            "int8_t" => FieldType::I8,
            "uint16_t" => FieldType::U16,
            "int16_t" => FieldType::I16,
            "uint32_t" => FieldType::U32,
            "int32_t" => FieldType::I32,
            "float" => FieldType::F32,
            "uint64_t" => FieldType::U64,
            "int64_t" => FieldType::I64,
            "double" => FieldType::F64,
            _ => return None,
        })
    }

    /// Size of one element on the wire, in bytes.
    pub fn size(self) -> usize {
        match self {
            FieldType::Char | FieldType::U8 | FieldType::I8 => 1,
            FieldType::U16 | FieldType::I16 => 2,
            FieldType::U32 | FieldType::I32 | FieldType::F32 => 4,
            FieldType::U64 | FieldType::I64 | FieldType::F64 => 8,
        }
    }

    /// The type name that goes into the `CRC_EXTRA` computation.
    pub fn crc_name(self) -> &'static str {
        match self {
            FieldType::Char => "char",
            FieldType::U8 => "uint8_t",
            FieldType::I8 => "int8_t",
            FieldType::U16 => "uint16_t",
            FieldType::I16 => "int16_t",
            FieldType::U32 => "uint32_t",
            FieldType::I32 => "int32_t",
            FieldType::F32 => "float",
            FieldType::U64 => "uint64_t",
            FieldType::I64 => "int64_t",
            FieldType::F64 => "double",
        }
    }
}

/// A message field, with its position in the MAVLink 2 payload.
#[derive(Clone, Debug, PartialEq)]
pub struct Field {
    /// Field name.
    pub name: String,
    /// Element type.
    pub ty: FieldType,
    /// Array length, or `None` for a scalar.
    pub array_len: Option<usize>,
    /// Enum that constrains the value, if any.
    pub enum_name: Option<String>,
    /// `units` attribute, if any.
    pub units: Option<String>,
    /// True for fields after `<extensions/>`.
    pub extension: bool,
    /// Byte offset in the payload.
    pub offset: usize,
    /// Field description text.
    pub description: String,
}

impl Field {
    /// Size on the wire, in bytes.
    pub fn size(&self) -> usize {
        self.ty.size() * self.array_len.unwrap_or(1)
    }
}

/// A message definition.
#[derive(Clone, Debug, PartialEq)]
pub struct Message {
    /// Message id.
    pub id: u32,
    /// Message name, for example `PARTICIPANT_POSITION`.
    pub name: String,
    /// Description text.
    pub description: String,
    /// Fields in wire order: base fields sorted by element size (largest
    /// first, stable), then extension fields in XML order.
    pub fields: Vec<Field>,
    /// `CRC_EXTRA` seed byte.
    pub crc_extra: u8,
    /// Payload length without extensions (the MAVLink 1 length).
    pub base_len: usize,
    /// Payload length with extensions.
    pub max_len: usize,
}

impl Message {
    /// The field called `name`.
    pub fn field(&self, name: &str) -> Option<&Field> {
        self.fields.iter().find(|f| f.name == name)
    }

    /// True if this message is out of scope (target, track, handover,
    /// fires, munition, arming, weapon, engagement or payload, see
    /// [`super::scope`]). Its fields are never decoded or encoded.
    pub fn is_blocked(&self) -> bool {
        super::scope::is_blocked(self.id, &self.name)
    }

    /// Decodes field `name` from `payload`. MAVLink 2 trims trailing zero
    /// bytes, so missing bytes read as zero. `None` for a blocked message
    /// ([`Message::is_blocked`]).
    pub fn get(&self, payload: &[u8], name: &str) -> Option<Value> {
        if self.is_blocked() {
            return None;
        }
        self.field(name).map(|f| decode_field(f, payload))
    }

    /// Decodes every field, in wire order. Empty for a blocked message
    /// ([`Message::is_blocked`]).
    pub fn decode(&self, payload: &[u8]) -> Vec<(&Field, Value)> {
        if self.is_blocked() {
            return Vec::new();
        }
        self.fields
            .iter()
            .map(|f| (f, decode_field(f, payload)))
            .collect()
    }

    /// Encodes `values` (by field name) into a full-length payload; fields
    /// not given are zero. Unknown names and wrong value kinds are errors,
    /// and so is a blocked message ([`Message::is_blocked`]).
    pub fn encode(&self, values: &[(&str, Value)]) -> Result<Vec<u8>, Error> {
        if self.is_blocked() {
            return Err(Error(format!(
                "{} is out of scope (targeting or engagement) and is not encoded",
                self.name
            )));
        }
        let mut out = vec![0u8; self.max_len];
        for (name, v) in values {
            let f = self
                .field(name)
                .ok_or_else(|| Error(format!("{} has no field {name}", self.name)))?;
            encode_field(f, v, &mut out[f.offset..f.offset + f.size()])
                .map_err(|e| Error(format!("{}.{name}: {e}", self.name)))?;
        }
        Ok(out)
    }
}

/// An enum entry.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry {
    /// Entry name.
    pub name: String,
    /// Entry value.
    pub value: u64,
}

/// An enum definition.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Enum {
    /// Enum name.
    pub name: String,
    /// True if `bitmask="true"`.
    pub bitmask: bool,
    /// Entries in XML order (entries from included files first).
    pub entries: Vec<Entry>,
    /// Description text.
    pub description: String,
}

impl Enum {
    /// The entry with `value`.
    pub fn entry(&self, value: u64) -> Option<&Entry> {
        self.entries.iter().find(|e| e.value == value)
    }

    /// OR of every entry value (for bitmask enums).
    pub fn all_bits(&self) -> u64 {
        self.entries.iter().fold(0, |a, e| a | e.value)
    }

    /// Smallest Hamming distance between two of the entries, if there are
    /// at least two. The MAVLink-M safety-critical enums keep this at 4 or
    /// more so that 1 to 3 flipped bits never produce another valid value.
    pub fn min_hamming_distance(&self) -> Option<u32> {
        let mut best = None::<u32>;
        for (i, a) in self.entries.iter().enumerate() {
            for b in &self.entries[i + 1..] {
                let d = (a.value ^ b.value).count_ones();
                best = Some(best.map_or(d, |x| x.min(d)));
            }
        }
        best
    }

    /// A readable label for `value`: the entry name without the enum's
    /// prefix, lower case with spaces (`MAVLINK_M_TARGET_FORCE_FOE` gives
    /// `foe`). `None` if the value is not defined.
    pub fn label(&self, value: u64) -> Option<String> {
        let e = self.entry(value)?;
        let own = format!("{}_", self.name);
        let prefix = if self.entries.iter().all(|x| x.name.starts_with(&own)) {
            own
        } else {
            self.common_prefix()
        };
        let short = e.name.strip_prefix(&prefix).unwrap_or(&e.name);
        let short = if short.is_empty() { &e.name } else { short };
        Some(short.to_ascii_lowercase().replace('_', " "))
    }

    /// Longest `_`-terminated prefix shared by every entry name (empty if
    /// there are fewer than two entries).
    fn common_prefix(&self) -> String {
        if self.entries.len() < 2 {
            return String::new();
        }
        let first = &self.entries[0].name;
        let mut n = first.len();
        for x in &self.entries[1..] {
            n = n.min(
                first
                    .bytes()
                    .zip(x.name.bytes())
                    .take_while(|(a, b)| a == b)
                    .count(),
            );
        }
        match first[..n].rfind('_') {
            Some(i) => first[..=i].to_string(),
            None => String::new(),
        }
    }
}

/// A decoded field value.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    /// Unsigned integer.
    Uint(u64),
    /// Signed integer.
    Int(i64),
    /// Floating point.
    Float(f64),
    /// `char[N]`, cut at the first NUL. Invalid UTF-8 is replaced.
    Text(String),
    /// Numeric array.
    Array(Vec<Value>),
}

impl Value {
    /// The value as `u64` (integers only; negative values give `None`).
    pub fn as_u64(&self) -> Option<u64> {
        match *self {
            Value::Uint(v) => Some(v),
            Value::Int(v) => u64::try_from(v).ok(),
            _ => None,
        }
    }

    /// The value as `i64` (integers only).
    pub fn as_i64(&self) -> Option<i64> {
        match *self {
            Value::Uint(v) => i64::try_from(v).ok(),
            Value::Int(v) => Some(v),
            _ => None,
        }
    }

    /// The value as `f64` (any number).
    pub fn as_f64(&self) -> Option<f64> {
        match *self {
            Value::Uint(v) => Some(v as f64),
            Value::Int(v) => Some(v as f64),
            Value::Float(v) => Some(v),
            _ => None,
        }
    }

    /// The text of a `char[N]` field.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Text(s) => Some(s),
            _ => None,
        }
    }

    /// A `uint8_t[N]` array as bytes.
    pub fn as_bytes(&self) -> Option<Vec<u8>> {
        match self {
            Value::Array(v) => v
                .iter()
                .map(|x| x.as_u64().and_then(|b| u8::try_from(b).ok()))
                .collect(),
            _ => None,
        }
    }
}

fn read_scalar(ty: FieldType, b: &[u8]) -> Value {
    let mut a = [0u8; 8];
    a[..b.len()].copy_from_slice(b);
    match ty {
        FieldType::Char | FieldType::U8 => Value::Uint(a[0] as u64),
        FieldType::I8 => Value::Int(a[0] as i8 as i64),
        FieldType::U16 => Value::Uint(u16::from_le_bytes([a[0], a[1]]) as u64),
        FieldType::I16 => Value::Int(i16::from_le_bytes([a[0], a[1]]) as i64),
        FieldType::U32 => Value::Uint(u32::from_le_bytes([a[0], a[1], a[2], a[3]]) as u64),
        FieldType::I32 => Value::Int(i32::from_le_bytes([a[0], a[1], a[2], a[3]]) as i64),
        FieldType::F32 => Value::Float(f32::from_le_bytes([a[0], a[1], a[2], a[3]]) as f64),
        FieldType::U64 => Value::Uint(u64::from_le_bytes(a)),
        FieldType::I64 => Value::Int(i64::from_le_bytes(a)),
        FieldType::F64 => Value::Float(f64::from_le_bytes(a)),
    }
}

/// Decodes one field; bytes past the end of `payload` read as zero.
pub fn decode_field(f: &Field, payload: &[u8]) -> Value {
    let mut buf = vec![0u8; f.size()];
    if f.offset < payload.len() {
        let n = (payload.len() - f.offset).min(buf.len());
        buf[..n].copy_from_slice(&payload[f.offset..f.offset + n]);
    }
    let es = f.ty.size();
    match (f.ty, f.array_len) {
        (FieldType::Char, Some(_)) => {
            let end = buf.iter().position(|&c| c == 0).unwrap_or(buf.len());
            Value::Text(String::from_utf8_lossy(&buf[..end]).into_owned())
        }
        (ty, Some(n)) => Value::Array(
            (0..n)
                .map(|i| read_scalar(ty, &buf[i * es..(i + 1) * es]))
                .collect(),
        ),
        (ty, None) => read_scalar(ty, &buf),
    }
}

fn encode_scalar(ty: FieldType, v: &Value, out: &mut [u8]) -> Result<(), String> {
    let int = || {
        v.as_i64()
            .or_else(|| v.as_u64().map(|u| u as i64))
            .ok_or_else(|| format!("expected an integer, got {v:?}"))
    };
    let float = || {
        v.as_f64()
            .ok_or_else(|| format!("expected a number, got {v:?}"))
    };
    match ty {
        FieldType::Char | FieldType::U8 | FieldType::I8 => out[0] = int()? as u8,
        FieldType::U16 | FieldType::I16 => out.copy_from_slice(&(int()? as u16).to_le_bytes()),
        FieldType::U32 | FieldType::I32 => out.copy_from_slice(&(int()? as u32).to_le_bytes()),
        FieldType::U64 => out.copy_from_slice(
            &v.as_u64()
                .ok_or_else(|| format!("expected an unsigned integer, got {v:?}"))?
                .to_le_bytes(),
        ),
        FieldType::I64 => out.copy_from_slice(&int()?.to_le_bytes()),
        FieldType::F32 => out.copy_from_slice(&(float()? as f32).to_le_bytes()),
        FieldType::F64 => out.copy_from_slice(&float()?.to_le_bytes()),
    }
    Ok(())
}

fn encode_field(f: &Field, v: &Value, out: &mut [u8]) -> Result<(), String> {
    let es = f.ty.size();
    match (f.ty, f.array_len, v) {
        (FieldType::Char, Some(n), Value::Text(s)) => {
            if s.len() > n {
                return Err(format!("text longer than {n} bytes"));
            }
            out[..s.len()].copy_from_slice(s.as_bytes());
            Ok(())
        }
        (_, Some(n), Value::Array(items)) => {
            if items.len() > n {
                return Err(format!("more than {n} elements"));
            }
            for (i, it) in items.iter().enumerate() {
                encode_scalar(f.ty, it, &mut out[i * es..(i + 1) * es])?;
            }
            Ok(())
        }
        (_, Some(_), _) => Err(format!("array field needs an array value, got {v:?}")),
        (_, None, _) => encode_scalar(f.ty, v, out),
    }
}

/// A dialect: its messages and enums, including those of the files it
/// includes when it was loaded with [`Dialect::from_file`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Dialect {
    /// Dialect name (the file stem, or `"military"` for the embedded copy).
    pub name: String,
    /// Messages by id.
    pub messages: BTreeMap<u32, Message>,
    /// Enums by name.
    pub enums: BTreeMap<String, Enum>,
    /// `<include>` file names that were not loaded.
    pub unresolved_includes: Vec<String>,
}

/// A dialect loading or encoding error.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Error(pub String);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

/// Parses an enum entry value: decimal, `0x` hex, `0b` binary or `2**n`.
fn parse_value(s: &str) -> Option<u64> {
    let s = s.trim();
    if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(h, 16).ok()
    } else if let Some(b) = s.strip_prefix("0b").or_else(|| s.strip_prefix("0B")) {
        u64::from_str_radix(b, 2).ok()
    } else if let Some(p) = s.strip_prefix("2**") {
        p.parse::<u32>().ok().and_then(|p| 1u64.checked_shl(p))
    } else {
        s.parse().ok()
    }
}

fn text_of(n: roxmltree::Node<'_, '_>) -> String {
    n.text()
        .unwrap_or("")
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
}

fn child_text(n: roxmltree::Node<'_, '_>, tag: &str) -> String {
    n.children()
        .find(|c| c.has_tag_name(tag))
        .map(text_of)
        .unwrap_or_default()
}

/// MAVLink's CRC-16/MCRF4XX step.
pub fn crc_accumulate(byte: u8, crc: u16) -> u16 {
    let mut tmp = byte ^ (crc & 0xff) as u8;
    tmp ^= tmp << 4;
    let t = tmp as u16;
    (crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)
}

fn crc_str(s: &str, mut crc: u16) -> u16 {
    for b in s.bytes() {
        crc = crc_accumulate(b, crc);
    }
    crc
}

/// Lays out a message: wire order, offsets, lengths and `CRC_EXTRA`.
pub fn layout(id: u32, name: &str, description: &str, xml_fields: Vec<Field>) -> Message {
    let (mut base, ext): (Vec<Field>, Vec<Field>) =
        xml_fields.into_iter().partition(|f| !f.extension);
    // stable sort, largest element first
    base.sort_by_key(|f| std::cmp::Reverse(f.ty.size()));
    let mut crc = crc_str(&format!("{name} "), 0xFFFF);
    for f in &base {
        crc = crc_str(&format!("{} ", f.ty.crc_name()), crc);
        crc = crc_str(&format!("{} ", f.name), crc);
        if let Some(n) = f.array_len {
            crc = crc_accumulate(n as u8, crc);
        }
    }
    let crc_extra = ((crc & 0xFF) ^ (crc >> 8)) as u8;
    let mut fields = base;
    fields.extend(ext);
    let mut off = 0;
    let mut base_len = 0;
    for f in &mut fields {
        f.offset = off;
        off += f.size();
        if !f.extension {
            base_len = off;
        }
    }
    Message {
        id,
        name: name.to_string(),
        description: description.to_string(),
        fields,
        crc_extra,
        base_len,
        max_len: off,
    }
}

impl Dialect {
    /// Parses one dialect XML document. `<include>` elements are listed in
    /// `unresolved_includes`, not loaded; use [`Dialect::from_file`] for that.
    pub fn parse(name: &str, xml: &str) -> Result<Self, Error> {
        let doc = roxmltree::Document::parse(xml).map_err(|e| Error(format!("{name}: {e}")))?;
        let root = doc.root_element();
        if !root.has_tag_name("mavlink") {
            return Err(Error(format!("{name}: root element is not <mavlink>")));
        }
        let mut d = Dialect {
            name: name.to_string(),
            ..Default::default()
        };
        for n in root.children().filter(|n| n.is_element()) {
            match n.tag_name().name() {
                "include" => d.unresolved_includes.push(text_of(n)),
                "enums" => {
                    for e in n.children().filter(|c| c.has_tag_name("enum")) {
                        d.add_enum(name, e)?;
                    }
                }
                "messages" => {
                    for m in n.children().filter(|c| c.has_tag_name("message")) {
                        let msg = parse_message(name, m)?;
                        d.messages.insert(msg.id, msg);
                    }
                }
                _ => {}
            }
        }
        Ok(d)
    }

    fn add_enum(&mut self, file: &str, e: roxmltree::Node<'_, '_>) -> Result<(), Error> {
        let ename = e
            .attribute("name")
            .ok_or_else(|| Error(format!("{file}: enum without a name")))?;
        let mut entries = Vec::new();
        for en in e.children().filter(|c| c.has_tag_name("entry")) {
            let n = en.attribute("name").unwrap_or("").to_string();
            let v = en
                .attribute("value")
                .and_then(parse_value)
                .ok_or_else(|| Error(format!("{file}: {ename}.{n}: missing or bad value")))?;
            entries.push(Entry { name: n, value: v });
        }
        let slot = self.enums.entry(ename.to_string()).or_insert_with(|| Enum {
            name: ename.to_string(),
            bitmask: false,
            entries: Vec::new(),
            description: String::new(),
        });
        slot.bitmask |= e.attribute("bitmask") == Some("true");
        if slot.description.is_empty() {
            slot.description = child_text(e, "description");
        }
        for en in entries {
            if !slot.entries.iter().any(|x| x.name == en.name) {
                slot.entries.push(en);
            }
        }
        Ok(())
    }

    /// Loads a dialect file and, recursively, the files it includes from the
    /// same directory. An include that is missing is listed in
    /// `unresolved_includes`.
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, Error> {
        let path = path.as_ref();
        let mut seen = Vec::new();
        load(path, &mut seen)
    }

    /// Adds the messages and enums of `other` (as an include does).
    pub fn merge(&mut self, other: Dialect) {
        for (id, m) in other.messages {
            self.messages.entry(id).or_insert(m);
        }
        for (name, e) in other.enums {
            match self.enums.get_mut(&name) {
                None => {
                    self.enums.insert(name, e);
                }
                Some(mine) => {
                    // included entries come first
                    let mut all = e.entries;
                    for x in std::mem::take(&mut mine.entries) {
                        if !all.iter().any(|y| y.name == x.name) {
                            all.push(x);
                        }
                    }
                    mine.entries = all;
                    mine.bitmask |= e.bitmask;
                }
            }
        }
    }

    /// The message with `id`.
    pub fn message(&self, id: u32) -> Option<&Message> {
        self.messages.get(&id)
    }

    /// The message called `name`.
    pub fn message_by_name(&self, name: &str) -> Option<&Message> {
        self.messages.values().find(|m| m.name == name)
    }

    /// `CRC_EXTRA` of message `id`.
    pub fn crc_extra(&self, id: u32) -> Option<u8> {
        self.message(id).map(|m| m.crc_extra)
    }

    /// Readable label of `value` in enum `enum_name` (see [`Enum::label`]).
    pub fn label(&self, enum_name: &str, value: u64) -> Option<String> {
        self.enums.get(enum_name)?.label(value)
    }
}

fn load(path: &Path, seen: &mut Vec<std::path::PathBuf>) -> Result<Dialect, Error> {
    let canon = path.canonicalize().unwrap_or_else(|_| path.to_path_buf());
    seen.push(canon);
    let xml =
        std::fs::read_to_string(path).map_err(|e| Error(format!("{}: {e}", path.display())))?;
    let stem = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let mut d = Dialect::parse(&stem, &xml)?;
    let dir = path.parent().unwrap_or_else(|| Path::new("."));
    let mut unresolved = Vec::new();
    for inc in std::mem::take(&mut d.unresolved_includes) {
        let p = dir.join(&inc);
        let c = p.canonicalize().unwrap_or_else(|_| p.clone());
        if seen.contains(&c) {
            continue;
        }
        if !p.is_file() {
            unresolved.push(inc);
            continue;
        }
        let sub = load(&p, seen)?;
        unresolved.extend(sub.unresolved_includes.iter().cloned());
        d.merge(sub);
    }
    d.unresolved_includes = unresolved;
    Ok(d)
}

fn parse_message(file: &str, m: roxmltree::Node<'_, '_>) -> Result<Message, Error> {
    let name = m
        .attribute("name")
        .ok_or_else(|| Error(format!("{file}: message without a name")))?;
    let id = m
        .attribute("id")
        .and_then(|v| v.trim().parse::<u32>().ok())
        .ok_or_else(|| Error(format!("{file}: {name}: missing or bad id")))?;
    let mut fields = Vec::new();
    let mut extension = false;
    for c in m.children().filter(|c| c.is_element()) {
        match c.tag_name().name() {
            "extensions" => extension = true,
            "field" => {
                let fname = c.attribute("name").unwrap_or("").to_string();
                let t = c.attribute("type").unwrap_or("");
                let (base, len) = match t.split_once('[') {
                    Some((b, rest)) => {
                        let n = rest
                            .trim_end_matches(']')
                            .parse::<usize>()
                            .map_err(|_| Error(format!("{file}: {name}.{fname}: bad type {t}")))?;
                        (b, Some(n))
                    }
                    None => (t, None),
                };
                let ty = FieldType::parse(base)
                    .ok_or_else(|| Error(format!("{file}: {name}.{fname}: unknown type {t}")))?;
                fields.push(Field {
                    name: fname,
                    ty,
                    array_len: len,
                    enum_name: c.attribute("enum").map(str::to_string),
                    units: c.attribute("units").map(str::to_string),
                    extension,
                    offset: 0,
                    description: text_of(c),
                });
            }
            _ => {}
        }
    }
    Ok(layout(id, name, &child_text(m, "description"), fields))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_extra_matches_known_common_values() {
        // HEARTBEAT and GLOBAL_POSITION_INT from common.xml; their CRC_EXTRA
        // values (50 and 104) are in every MAVLink implementation.
        let xml = r#"<mavlink><messages>
          <message id="0" name="HEARTBEAT">
            <field type="uint8_t" name="type">t</field>
            <field type="uint8_t" name="autopilot">a</field>
            <field type="uint8_t" name="base_mode">b</field>
            <field type="uint32_t" name="custom_mode">c</field>
            <field type="uint8_t" name="system_status">s</field>
            <field type="uint8_t_mavlink_version" name="mavlink_version">v</field>
          </message>
          <message id="33" name="GLOBAL_POSITION_INT">
            <field type="uint32_t" name="time_boot_ms">t</field>
            <field type="int32_t" name="lat">a</field>
            <field type="int32_t" name="lon">a</field>
            <field type="int32_t" name="alt">a</field>
            <field type="int32_t" name="relative_alt">a</field>
            <field type="int16_t" name="vx">a</field>
            <field type="int16_t" name="vy">a</field>
            <field type="int16_t" name="vz">a</field>
            <field type="uint16_t" name="hdg">a</field>
          </message></messages></mavlink>"#;
        let d = Dialect::parse("t", xml).unwrap();
        assert_eq!(d.crc_extra(0), Some(50));
        assert_eq!(d.crc_extra(33), Some(104));
        let hb = d.message(0).unwrap();
        assert_eq!(hb.fields[0].name, "custom_mode");
        assert_eq!(hb.base_len, 9);
    }

    #[test]
    fn values() {
        assert_eq!(parse_value("0x0303"), Some(0x0303));
        assert_eq!(parse_value("2**3"), Some(8));
        assert_eq!(parse_value("0b101"), Some(5));
        assert_eq!(parse_value(" 42 "), Some(42));
        assert_eq!(parse_value("x"), None);
    }
}
